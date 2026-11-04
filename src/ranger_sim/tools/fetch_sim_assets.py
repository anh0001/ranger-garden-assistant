#!/usr/bin/env python3
"""
Fetch the pinned third-party sim assets into models/third_party/ (git-ignored).

  python3 src/ranger_sim/tools/fetch_sim_assets.py           # then rebuild ranger_sim
  python3 src/ranger_sim/tools/fetch_sim_assets.py --force   # download everything again

What is fetched is pinned in tools/sim_assets.yaml:
  - the AWS RoboMaker Small House furniture models (MIT-0), from TeamSOBITS'
    Humble/Fortress port at a fixed commit, into third_party/aws_small_house/
    with the repo's own layout (models/ next to photos/: the portrait meshes
    load their pictures from ../../../photos);
  - Google Scanned Objects from Gazebo Fuel (CC BY 4.0), each at a fixed
    version and checked against its sha256 before it is unpacked, into
    third_party/<name>/.
gazebo.launch.py puts third_party/ and third_party/*/models on the Gazebo
resource path. Attribution: models/THIRD_PARTY_ASSETS.md.

Each asset is unpacked next to its final directory and moved into place only
when complete, with a stamp of its pinned commit / sha256 (STAMP_FILE). An
asset whose stamp matches the manifest is skipped, so the tool can be re-run
safely; an interrupted run, a changed pin or a deleted directory fetches
that asset again.
"""

import argparse
import hashlib
import io
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

from PIL import Image
import yaml

PKG = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = PKG / "tools" / "sim_assets.yaml"
GSO_URL = "https://fuel.gazebosim.org/1.0/GoogleResearch/models/{name}/{version}/{name}.zip"
DOWNLOAD_TIMEOUT_SEC = 60
GIT_TIMEOUT_SEC = 600  # the house fetch is ~80 MB
HOUSE_DIR = "aws_small_house"  # under target_dir; holds models/, photos/, LICENSE
HOUSE_MODEL_GLOB = "aws_robomaker_residential_*"
GSO_KEYS = ("name", "version", "wrapper", "sha256")
ASSET_NAME = re.compile(r"[A-Za-z0-9_]+")  # names become directories and URL parts
STAMP_FILE = ".fetched"  # in each installed asset directory: its pinned commit / sha256


def gso_url(name: str, version: int) -> str:
    """Gazebo Fuel download URL of one Google Scanned Objects model version."""
    return GSO_URL.format(name=name, version=int(version))


def load_manifest(path: Path = DEFAULT_MANIFEST) -> dict:
    """Read tools/sim_assets.yaml and check that every download is pinned."""
    manifest = yaml.safe_load(Path(path).read_text())
    if not isinstance(manifest, dict):
        raise ValueError(f"{path}: expected a mapping")
    for key in ("target_dir", "texture_max_px", "aws_small_house", "gso"):
        if key not in manifest:
            raise ValueError(f"{path}: missing '{key}'")
    house = manifest["aws_small_house"]
    if not house.get("repo") or len(str(house.get("commit", ""))) != 40:
        raise ValueError(f"{path}: aws_small_house needs a repo and a full 40-hex commit")
    for index, entry in enumerate(manifest["gso"]):
        missing = [key for key in GSO_KEYS if key not in entry]
        if missing:
            raise ValueError(f"{path}: gso[{index}] is missing {', '.join(missing)}")
        if len(str(entry["sha256"])) != 64:
            raise ValueError(f"{path}: gso[{index}] sha256 must be 64 hex digits")
        for key in ("name", "wrapper"):
            if not ASSET_NAME.fullmatch(str(entry[key])):
                raise ValueError(
                    f"{path}: gso[{index}] {key} {entry[key]!r} must match {ASSET_NAME.pattern}"
                )
    return manifest


def verify_sha256(data: bytes, expected: str, label: str) -> None:
    """Raise ValueError unless `data` hashes to `expected`."""
    actual = hashlib.sha256(data).hexdigest()
    if actual != expected.lower():
        raise ValueError(f"sha256 mismatch for {label}: expected {expected}, got {actual}")


def _check_member(name: str) -> None:
    path = PurePosixPath(name.replace("\\", "/"))
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"unsafe path in archive: {name!r}")


def safe_extract(zip_bytes: bytes, dest: Path) -> None:
    """Unpack a zip into `dest`; refuse the whole archive if any member escapes it."""
    with zipfile.ZipFile(io.BytesIO(zip_bytes)) as archive:
        members = archive.infolist()
        for member in members:  # check all first: a bad archive extracts nothing
            _check_member(member.filename)
        dest.mkdir(parents=True, exist_ok=True)
        for member in members:
            archive.extract(member, dest)


def place_texture(model_dir: Path, max_px: int) -> Path:
    """
    Write a downscaled copy of the scan's texture to meshes/texture.png.

    The scan's meshes/model.mtl says `map_Kd texture.png`, i.e. next to the
    mtl, while the Fuel zip ships it as materials/textures/texture.png
    (4096^2). The original is kept.
    """
    source = model_dir / "materials" / "textures" / "texture.png"
    if not source.is_file():
        raise RuntimeError(f"no texture at {source}")
    target = model_dir / "meshes" / "texture.png"
    target.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(source) as image:
        image.thumbnail((max_px, max_px))
        image.save(target, format="PNG")
    return target


def _git(*args: str, cwd: Path) -> str:
    try:
        result = subprocess.run(
            ["git", *args], cwd=cwd, check=True, capture_output=True, text=True,
            timeout=GIT_TIMEOUT_SEC,
        )
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"git {args[0]} failed: {exc.stderr.strip()}") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"git {args[0]} timed out after {GIT_TIMEOUT_SEC} s") from exc
    return result.stdout.strip()


def _installed(asset_dir: Path, pin: str) -> bool:
    """Whether `asset_dir` holds a complete install of the asset pinned to `pin`."""
    stamp = asset_dir / STAMP_FILE
    return stamp.is_file() and stamp.read_text().strip() == pin


def _staging(target: Path, name: str) -> Path:
    """Fresh hidden directory next to target/`name` to build the asset in."""
    staging = target / f".{name}.partial"
    if staging.exists():
        shutil.rmtree(staging)  # left over from an interrupted run
    return staging


def _install(staging: Path, asset_dir: Path, pin: str) -> None:
    """Stamp a complete `staging` with `pin` and move it to `asset_dir`."""
    (staging / STAMP_FILE).write_text(pin + "\n")
    if asset_dir.exists():
        shutil.rmtree(asset_dir)
    staging.rename(asset_dir)


def fetch_house(repo: str, commit: str, target: Path) -> int:
    """
    Install the house of `repo` at `commit` as `target`/aws_small_house.

    models/aws_robomaker_residential_*, photos/ and LICENSE keep the repo's
    layout; returns the number of models.
    """
    staging = _staging(target, HOUSE_DIR)
    with tempfile.TemporaryDirectory() as tmp:
        checkout = Path(tmp)
        _git("init", "-q", cwd=checkout)
        _git("fetch", "-q", "--depth", "1", "--", repo, commit, cwd=checkout)
        _git("-c", "advice.detachedHead=false", "checkout", "-q", "FETCH_HEAD", cwd=checkout)
        head = _git("rev-parse", "HEAD", cwd=checkout)
        if head != commit:
            raise RuntimeError(f"{repo}: checked out {head}, expected {commit}")
        models = sorted((checkout / "models").glob(HOUSE_MODEL_GLOB))
        if not models:
            raise RuntimeError(f"{repo}@{commit}: no models/{HOUSE_MODEL_GLOB}")
        for model in models:
            shutil.copytree(model, staging / "models" / model.name)
        shutil.copytree(checkout / "photos", staging / "photos")
        shutil.copyfile(checkout / "LICENSE", staging / "LICENSE")
    _install(staging, target / HOUSE_DIR, commit)
    return len(models)


def _download(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=DOWNLOAD_TIMEOUT_SEC) as response:  # noqa: S310
        return response.read()


def _fetch_gso(entry: dict, target: Path, max_px: int) -> None:
    name = entry["name"]
    url = gso_url(name, entry["version"])
    try:
        data = _download(url)
    except OSError as exc:  # URLError, HTTPError, timeouts
        raise RuntimeError(f"{name}: download of {url} failed: {exc}") from exc
    verify_sha256(data, entry["sha256"], name)
    staging = _staging(target, name)
    safe_extract(data, staging)
    place_texture(staging, max_px)
    _install(staging, target / name, entry["sha256"])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--force", action="store_true", help="download assets already present")
    args = parser.parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        target = PKG / manifest["target_dir"]
        house = manifest["aws_small_house"]
        if _installed(target / HOUSE_DIR, house["commit"]) and not args.force:
            print(f"aws_small_house @ {house['commit'][:7]}: already present")
        else:
            count = fetch_house(house["repo"], house["commit"], target)
            print(
                f"aws_small_house @ {house['commit'][:7]}: {count} models -> "
                f"{target / HOUSE_DIR}"
            )
        for entry in manifest["gso"]:
            if _installed(target / entry["name"], entry["sha256"]) and not args.force:
                print(f"{entry['name']}: already present")
                continue
            _fetch_gso(entry, target, int(manifest["texture_max_px"]))
            print(f"{entry['name']} v{entry['version']}: sha256 ok -> {target / entry['name']}")
    except Exception as exc:  # noqa: BLE001 - CLI: report any failure and exit non-zero
        print(f"fetch_sim_assets: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
