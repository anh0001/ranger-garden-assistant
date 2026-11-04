# Third-party simulation assets

The `small_house` scene (`worlds/small_house_world.sdf`) uses third-party
models. They are **fetched, not committed**: run

```bash
python3 src/ranger_sim/tools/fetch_sim_assets.py   # then rebuild ranger_sim
```

which downloads the pinned versions listed in `tools/sim_assets.yaml` into
`models/third_party/` (git-ignored).

## AWS RoboMaker Small House World

- Copyright 2019 Amazon.com, Inc. or its affiliates. Licensed under MIT-0
  (MIT No Attribution); the licence text is fetched to
  `models/third_party/aws_small_house/LICENSE`.
- Source: the Humble / Gazebo Fortress port
  [TeamSOBITS/aws_small_house_world](https://github.com/TeamSOBITS/aws_small_house_world)
  at commit `e2f90621f0fd1a0c496f4e7fcca06138808bc2ec` (branch
  `humble-devel`, 2025-09-05), from the original
  [aws-robotics/aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world).
  That repository's LICENSE file is MIT-0; its `package.xml` says
  Apache 2.0 and its README says MIT, and the LICENSE file is what applies.
- Fetched: the 68 `aws_robomaker_residential_*` furniture models and the
  `photos/` they texture the portraits with, kept in the repository's layout
  under `models/third_party/aws_small_house/`.
- Committed: `tools/aws_small_house/small_house.world`, a verbatim copy of
  the source world at that commit, with its `LICENSE`. The scene's world is
  generated from it by `tools/generate_small_house_world.py` (the house is
  moved rigidly; the kitchen table and one chair are replaced or dropped).

## Google Scanned Objects

- Copyright 2020 Google LLC. Licensed under the
  [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/)
  (CC BY 4.0).
- Source: Gazebo Fuel, owner `GoogleResearch`
  (`https://fuel.gazebosim.org/1.0/GoogleResearch/models/<name>/<version>`),
  each download checked against the sha256 in `tools/sim_assets.yaml`.
- Changes: the fetch tool adds a 1024 px copy of each texture next to the
  mesh (`meshes/texture.png`); the original is kept. The scans are used
  unmodified as visuals by wrapper models in this package, which add a
  primitive collision and inertia:

| Scan (Gazebo Fuel model, version 1) | Wrapper model |
|---|---|
| `AllergenFree_JarroDophilus` | `models/vitamin_bottle` |
| `Beta_Glucan` | `models/supplement_bottle` |
| `Wilton_Pearlized_Sugar_Sprinkles_525_oz_Gold` | `models/sprinkles_jar` |
