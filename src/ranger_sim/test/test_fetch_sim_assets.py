"""Tests for tools/fetch_sim_assets.py (offline: no network, no models/third_party)."""

import hashlib
import importlib.util
import io
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import zipfile

from PIL import Image
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
MANIFEST = PKG / "tools" / "sim_assets.yaml"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fetch = _load("fetch_sim_assets", PKG / "tools" / "fetch_sim_assets.py")


def _zip(members):
    """In-memory zip of {member name: bytes}."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        for name, data in members.items():
            archive.writestr(name, data)
    return buffer.getvalue()


def _png(size):
    buffer = io.BytesIO()
    Image.new("RGB", (size, size), (200, 40, 40)).save(buffer, format="PNG")
    return buffer.getvalue()


def test_verify_sha256_accepts_the_matching_digest():
    data = b"scanned object"
    fetch.verify_sha256(data, hashlib.sha256(data).hexdigest(), "x")


def test_verify_sha256_rejects_a_mismatch():
    with pytest.raises(ValueError, match="sha256 mismatch for Beta_Glucan"):
        fetch.verify_sha256(b"tampered", "0" * 64, "Beta_Glucan")


@pytest.mark.parametrize("member", ["../evil", "meshes/../../evil", "/etc/evil"])
def test_safe_extract_rejects_path_traversal(tmp_path, member):
    archive = _zip({"model.sdf": b"<sdf/>", member: b"payload"})
    dest = tmp_path / "model"

    with pytest.raises(ValueError, match="unsafe path"):
        fetch.safe_extract(archive, dest)
    assert not (tmp_path / "evil").exists()
    assert not (dest / "model.sdf").exists(), "nothing may be extracted from a bad archive"


def test_safe_extract_writes_the_members(tmp_path):
    archive = _zip({"model.sdf": b"<sdf/>", "meshes/model.obj": b"v 0 0 0"})

    fetch.safe_extract(archive, tmp_path / "model")

    assert (tmp_path / "model" / "meshes" / "model.obj").read_bytes() == b"v 0 0 0"


def test_place_texture_writes_a_downscaled_copy_next_to_the_mtl(tmp_path):
    archive = _zip({
        "materials/textures/texture.png": _png(64),
        "meshes/model.mtl": b"newmtl material_0\nmap_Kd texture.png\n",
    })
    model_dir = tmp_path / "Beta_Glucan"
    fetch.safe_extract(archive, model_dir)

    placed = fetch.place_texture(model_dir, max_px=16)

    assert placed == model_dir / "meshes" / "texture.png"
    with Image.open(placed) as image:
        assert image.size == (16, 16)
    with Image.open(model_dir / "materials" / "textures" / "texture.png") as original:
        assert original.size == (64, 64), "the original texture must be kept"


def test_place_texture_without_a_texture_is_an_error(tmp_path):
    with pytest.raises(RuntimeError, match="texture"):
        fetch.place_texture(tmp_path, max_px=16)


def test_gso_url_is_pinned_to_the_version():
    assert fetch.gso_url("Beta_Glucan", 1) == (
        "https://fuel.gazebosim.org/1.0/GoogleResearch/models/Beta_Glucan/1/Beta_Glucan.zip"
    )


def test_manifest_pins_the_house_commit_and_every_download():
    manifest = fetch.load_manifest(MANIFEST)

    assert len(manifest["aws_small_house"]["commit"]) == 40
    for entry in manifest["gso"]:
        assert len(entry["sha256"]) == 64, entry["name"]
        assert int(entry["version"]) >= 1, entry["name"]


def test_load_manifest_rejects_a_gso_entry_without_a_digest(tmp_path):
    manifest = yaml.safe_load(MANIFEST.read_text())
    del manifest["gso"][0]["sha256"]
    path = tmp_path / "sim_assets.yaml"
    path.write_text(yaml.safe_dump(manifest))

    with pytest.raises(ValueError, match="sha256"):
        fetch.load_manifest(path)


@pytest.mark.parametrize("broken, message", [
    ("[]", "expected a mapping"),
    ("target_dir: x\n", "missing 'texture_max_px'"),
    ("target_dir: x\ntexture_max_px: 8\ngso: []\naws_small_house: {repo: r, commit: abc}\n",
     "40-hex commit"),
])
def test_load_manifest_rejects_a_malformed_manifest(tmp_path, broken, message):
    path = tmp_path / "sim_assets.yaml"
    path.write_text(broken)

    with pytest.raises(ValueError, match=message):
        fetch.load_manifest(path)


# ---- end to end, offline: a local git repo and a stubbed download ---------


def _git(cwd, *args):
    return subprocess.run(
        ["git", *args], cwd=cwd, check=True, capture_output=True, text=True
    ).stdout.strip()


@pytest.fixture
def house_repo(tmp_path):
    """Local stand-in for the TeamSOBITS repo: (path, commit)."""
    repo = tmp_path / "house_repo"
    model = repo / "models" / "aws_robomaker_residential_Table_01"
    model.mkdir(parents=True)
    (model / "model.sdf").write_text("<sdf/>")
    (repo / "photos").mkdir()
    (repo / "photos" / "A.jpg").write_bytes(b"jpg")
    (repo / "LICENSE").write_text("MIT-0")
    _git(repo, "init", "-q")
    _git(repo, "config", "uploadpack.allowReachableSHA1InWant", "true")
    _git(repo, "add", ".")
    _git(repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "house")
    return repo, _git(repo, "rev-parse", "HEAD")


def test_fetch_house_keeps_the_repo_layout(tmp_path, house_repo):
    repo, commit = house_repo
    target = tmp_path / "third_party"

    count = fetch.fetch_house(str(repo), commit, target)

    house = target / fetch.HOUSE_DIR
    assert count == 1
    assert (house / "models" / "aws_robomaker_residential_Table_01" / "model.sdf").is_file()
    assert (house / "photos" / "A.jpg").is_file(), "portrait meshes load ../../../photos"
    assert (house / "LICENSE").read_text() == "MIT-0"
    assert (house / fetch.STAMP_FILE).read_text().strip() == commit


def test_fetch_house_rejects_a_repo_without_house_models(tmp_path, house_repo):
    repo, _ = house_repo
    _git(repo, "rm", "-q", "-r", "models")
    _git(repo, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-q", "-m", "empty")

    with pytest.raises(RuntimeError, match="no models/"):
        fetch.fetch_house(str(repo), _git(repo, "rev-parse", "HEAD"), tmp_path / "t")


def test_a_failing_git_fetch_says_why(tmp_path):
    with pytest.raises(RuntimeError, match="git fetch failed: .*not appear to be a git repo"):
        fetch.fetch_house(str(tmp_path / "no_such_repo"), "0" * 40, tmp_path / "t")


def _scan_zip(colour=(200, 40, 40)):
    image = io.BytesIO()
    Image.new("RGB", (64, 64), colour).save(image, format="PNG")
    return _zip({
        "model.sdf": b"<sdf/>",
        "materials/textures/texture.png": image.getvalue(),
        "meshes/model.mtl": b"map_Kd texture.png\n",
    })


@pytest.fixture
def offline_fetch(tmp_path, house_repo, monkeypatch):
    """Return a local house and one stubbed GSO download behind a manifest."""
    repo, commit = house_repo
    scan = _scan_zip()
    manifest = {
        "target_dir": "third_party",
        "texture_max_px": 16,
        "aws_small_house": {"repo": str(repo), "commit": commit},
        "gso": [{"name": "Scan", "version": 1, "wrapper": "scan_wrapper",
                 "sha256": hashlib.sha256(scan).hexdigest()}],
    }
    path = tmp_path / "sim_assets.yaml"
    path.write_text(yaml.safe_dump(manifest))
    state = SimpleNamespace(
        args=["--manifest", str(path)], downloads=[], scan=scan,
        manifest=manifest, path=path, target=tmp_path / "third_party",
    )
    monkeypatch.setattr(fetch, "PKG", tmp_path)
    monkeypatch.setattr(fetch, "_download", lambda url: state.downloads.append(url) or state.scan)
    return state


def test_main_fetches_everything_then_skips_what_is_present(offline_fetch, capsys):
    assert fetch.main(offline_fetch.args) == 0
    assert (offline_fetch.target / "Scan" / "meshes" / "texture.png").is_file()
    assert offline_fetch.downloads == [fetch.gso_url("Scan", 1)]

    assert fetch.main(offline_fetch.args) == 0
    assert len(offline_fetch.downloads) == 1, "a re-run must not download again"
    assert capsys.readouterr().out.count("already present") == 2


def test_main_exits_1_on_a_corrupted_download_and_extracts_nothing(
    offline_fetch, monkeypatch, capsys
):
    monkeypatch.setattr(fetch, "_download", lambda url: b"corrupted")

    assert fetch.main(offline_fetch.args) == 1
    assert "sha256 mismatch for Scan" in capsys.readouterr().err
    assert not (offline_fetch.target / "Scan").exists()


def test_an_interrupted_install_does_not_count_as_installed(offline_fetch, monkeypatch):
    def interrupted(model_dir, max_px):
        raise KeyboardInterrupt  # Ctrl-C while the texture is downscaled

    with monkeypatch.context() as patch:
        patch.setattr(fetch, "place_texture", interrupted)
        with pytest.raises(KeyboardInterrupt):
            fetch.main(offline_fetch.args)
    assert not (offline_fetch.target / "Scan").exists()

    assert fetch.main(offline_fetch.args) == 0
    assert (offline_fetch.target / "Scan" / "meshes" / "texture.png").is_file()
    assert len(offline_fetch.downloads) == 2


def test_a_new_pin_is_fetched_again(offline_fetch):
    assert fetch.main(offline_fetch.args) == 0
    offline_fetch.scan = _scan_zip(colour=(10, 200, 10))
    offline_fetch.manifest["gso"][0].update(
        version=2, sha256=hashlib.sha256(offline_fetch.scan).hexdigest()
    )
    offline_fetch.path.write_text(yaml.safe_dump(offline_fetch.manifest))

    assert fetch.main(offline_fetch.args) == 0
    assert offline_fetch.downloads[-1] == fetch.gso_url("Scan", 2)
    with Image.open(offline_fetch.target / "Scan" / "meshes" / "texture.png") as image:
        assert image.getpixel((0, 0)) == (10, 200, 10)


def test_a_deleted_house_is_fetched_again(offline_fetch):
    assert fetch.main(offline_fetch.args) == 0
    shutil.rmtree(offline_fetch.target / fetch.HOUSE_DIR)

    assert fetch.main(offline_fetch.args) == 0
    assert (offline_fetch.target / fetch.HOUSE_DIR / "LICENSE").is_file()


@pytest.mark.parametrize("field, value", [("name", "../escape"), ("wrapper", "a/b")])
def test_load_manifest_rejects_an_unsafe_asset_name(tmp_path, field, value):
    manifest = yaml.safe_load(MANIFEST.read_text())
    manifest["gso"][0][field] = value
    path = tmp_path / "sim_assets.yaml"
    path.write_text(yaml.safe_dump(manifest))

    with pytest.raises(ValueError, match=field):
        fetch.load_manifest(path)


def test_manifest_pins_every_wrapper_model():
    for entry in fetch.load_manifest(MANIFEST)["gso"]:
        wrapper = PKG / "models" / entry["wrapper"] / "model.sdf"
        assert wrapper.is_file(), f"missing wrapper model {wrapper}"
        uris = [u.text for u in ET.parse(wrapper).getroot().iter("uri")]
        assert any(u.startswith(f"model://{entry['name']}/") for u in uris), (
            f"{wrapper.name} of {entry['wrapper']} does not use the scan {entry['name']}"
        )
