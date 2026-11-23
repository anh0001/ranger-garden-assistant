# Whole-body MPC pick in the pick-and-place demo (vs the scripted pick)

The sim's pick-and-place demo can now do its pick with the whole-body MPC
([sim_whole_body_mpc.md](sim_whole_body_mpc.md)) instead of the scripted
sequence. The scripted pick needs the base within 5 mm and 0.005 rad of a
precomputed pose, because the arm only plays back joint waypoints. The MPC pick
moves the base and the arm together from wherever the base stopped. Both picks
can start from a deliberately wrong base pose, and each run logs one line
with the time and the gripper's alignment, so the two can be compared on the
same starts. The scripted pick stays the default. Gazebo Fortress only
(`ranger_sim`): nothing here has a real-robot path.

Video (house scene, recorded 2026-11-23 with
`record_pick_place_video.launch.py approach:=mpc approach_error:="[-0.08, 0.06, 0.07]"`):
[sim_wbmpc_pick_small_house.mp4](sim_wbmpc_pick_small_house.mp4), 130 s of
sim time, 13.8 MB.

- The base stops 8 cm short of the standoff, 6 cm to the left and turned 4°.
- The MPC takes over: pre-grasp, then the slide in. At 0:16 the wrist inset
  has the bottle centred, close up.
- Attach, close, lift (0:19), hand back.
- Then the unchanged transfer: to the kitchen counter, place, return.

## What changes in the pick

| Step | `approach:=sequential` (default) | `approach:=mpc` |
|---|---|---|
| 1 | drive to the table (the pick standoff, moved by `approach_error`) | same |
| 2 | correct the standoff (only with a non-zero error) | base to the whole-body MPC, arm to streamed setpoints |
| 3 | look at the object (MoveIt: grasp joints) | MPC: pre-grasp (grasp height, 0.15 m short of the grasp) |
| 4 | approach the object (base slides 0.15 m at 0.08 m/s) | MPC: slide in (the target moves 0.15 m along the approach at 0.03 m/s) |
| 5 | attach object, close gripper | same (the MPC holds in REACHED) |
| 6 | lift (MoveIt: hover joints) | MPC: lift (the target moves straight up to hover height) |
| 7 | carry pose | arm back to MoveIt, base back to navigation, carry pose |

The route, the place and the return are the same in both modes. With no
error, the sequential steps and captions are exactly the old ones.

Why some steps are as they are:
- **No corrective leg in MPC mode.** The MPC picks from wherever the base
  stopped.
- **The lift is the MPC's.** After an MPC pick the base is not at the pose
  the hover joints assume, so a joint-space lift would swing the held object
  sideways by the base's offset.
- **The slide streams a moving target.** The MPC minimises the pose error;
  it does not promise a straight TCP path. A target that moves slowly along
  the approach keeps the TCP on the line.

Targets are in `odom`, with the gripper level and the approach along the pick heading:
- `grasp` = pick base pose + `grasp_tcp`. House (0.965, 0, 0.871),
  garden (0.950, 0, 0.842).
- `pre-grasp` = `grasp` moved back by `standoff_m` (0.15 m).
- `lift` = `grasp` at `hover_tcp`'s height. House z 0.962, garden 0.935.

## Running it

```bash
# The MPC pick needs the gate (the scripted one works with or without it)
ros2 launch ranger_sim sim_bringup.launch.py handover_gate:=true
# wait for "Configured and activated piper_arm_controller", then:

ros2 launch ranger_sim pick_place_demo.launch.py execute:=true                  # scripted
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true approach:=mpc    # MPC pick

# From a wrong standoff, pick only: set the object back, return, 'home'
ros2 launch ranger_sim pick_place_demo.launch.py execute:=true approach:=mpc \
    approach_only:=true approach_error:="[-0.08, 0.06, 0.07]"

# Video (fresh sim)
ros2 launch ranger_sim record_pick_place_video.launch.py approach:=mpc \
    approach_error:="[-0.08, 0.06, 0.07]"

# Launch test: MPC pick from the wrong standoff (boots its own sim; GPU)
RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_wbmpc_pick_sim.py
```

Without `execute:=true` both modes only plan the arm moves: no motion, no
targets, nothing switched.

### Arguments (`pick_place_demo.launch.py`, `record_pick_place_video.launch.py`)

**`approach`** (`sequential` | `mpc`, default `sequential`):
- `sequential` starts `pick_place_demo.py`.
- `mpc` starts `wbmpc_pick_place_demo.py` and `whole_body_mpc`.
- Both demo nodes are named `pick_place_demo`, so the video captions work for
  either. The launch ends when the demo exits.

**`approach_error`** (`[dx, dy, dyaw]`, default `[0.0, 0.0, 0.0]`): the first
leg ends this far off the pick standoff.
- dx and dy are along and across the pick heading (m); dyaw is in rad.
- It is the stand-in for a coarse navigation stop, so it must stay within
  Nav2's goal tolerances: 0.25 m and 0.10 rad (`config/nav2_sim.yaml`).
- dx must stay short of the pick approach.
- Anything else, including NaN, fails at start with `ValueError`.

**`approach_only`** (default `false`): end the run after the lift.
- Set the object back down, open, release and back off. The MPC does this
  in reverse with its own lift and slide.
- Carry pose, return to start, `'home'`. `'home'` is below the table top and
  is only used away from the tables.

### Scene keys and MPC settings

New required keys per scene (`config/pick_place_scenes.yaml`):
- `grasp_tcp`, `hover_tcp`: `[x, z]` of the TCP in `base_footprint` at the
  grasp and hover joints. `test_pick_place_geometry.py` pins them to the FK
  within 0.5 mm.
- `mpc_base_box`: `[x_min, y_min, x_max, y_max]` in odom. `load_scenes`
  rejects one past `base_approach_x`. `test_pick_place_geometry.py` checks that
  the box contains the standoff and the origin, and that a base anywhere in it,
  at ±0.10 rad of yaw, keeps 3 cm to every collision box of the table. Values:
  - House: `[-0.40, -0.20, 0.52, 0.30]`, the box of `config/wbmpc.yaml`.
  - Garden: `[-0.40, -0.30, 0.50, 0.30]`. At the planned `x_max` = 0.52 the
    lidar came within 1.1 cm of a table leg at |y| ≥ 0.15 m and 0.1 rad of
    yaw. At 0.50 the minimum is 3.1 cm.

For the pick the launch overrides three `whole_body_mpc` parameters:
- `q_nominal` = the scene's `grasp_joints`. The arm ends near the proven grasp
  configuration; with the carry pose as posture, the planning spike had the
  base run to its box limit.
- `base_box` = `mpc_base_box`.
- `position_tolerance` = 3 mm instead of 5. With 2 mm, one start in fourteen
  sat at 2.2 mm in the planning spike.

Constants in the scripts:
- `SLIDE_SPEED_MPS` = 0.03 (`wbmpc_pick_place_demo.py`). The spike had the TCP
  within 1.7 mm of the line sideways at this speed; the open fingers clear the
  object by 8–10 mm per side.
- `SLIDE_PERIOD_SEC` = 0.1: a new target every two MPC ticks.
- `TARGET_SETTLE_SEC` = 0.25 s of sim time (`wbmpc_reach_demo.py`).
  `whole_body_mpc` reports its state on change only, so after the last target
  of a slide it may stay REACHED without a word. The demo lets it tick on the
  end target for this long, then trusts the latest state.

### The result line

Logged by the demo node right after the attach, in both modes:

```
approach result: mode=mpc time=7.4 s tcp offset=(-1.4, +0.0, -0.3) mm yaw=+4.2 deg aligned=yes
```

- **time** is on the node clock (sim time), from the end of the first leg to
  the attach.
- **tcp offset** is the TCP (TF `odom → piper_tcp`) minus the grasp target, in
  mm, along the pick heading, across it, and up.
- **yaw** is the base's heading off the pick heading.
- **aligned** means the offset is inside the finger, palm and pad clearances
  of `test_pick_place_geometry.py`:
  - −10 ≤ dx ≤ +5 mm (along: short, the pads still span the object; past, the
    palm stays ≥ 5 mm off it)
  - |dy| ≤ 6 mm
  - |dz| ≤ 10 mm
- A missing or stale transform gives a warning (and `tcp offset=n/a
  aligned=n/a` when there is none). The pick goes on.

## Comparison (2026-11-23, house, RTX 6000 Ada workstation)

Setup:
- `approach_only:=true`, one fresh headless sim per run, the errors of the plan.
- **Times** are sim time from the result line.
- **"Object after"** is how far the bottle stands from where it started once it
  has been set back (Gazebo model pose, xy). It bounds any push before the
  attach plus the set-down error.

| `approach_error` [dx, dy, dyaw] | Mode | Completed | Aligned | TCP offset dx, dy, dz (mm) | Approach (s) | Object after (mm) |
|---|---|---|---|---|---|---|
| [0, 0, 0] | sequential | yes | yes | −6.0, +0.3, +1.0 | 4.8 | 4.7 |
| [0, 0, 0] | mpc | yes | yes | −2.1, −0.0, −0.3 | 7.1 | 1.8 |
| [−0.10, 0, 0] | sequential | yes | yes | −6.0, +2.5, +5.7 | 7.2 | 4.6 |
| [−0.10, 0, 0] | mpc | yes | yes | −1.4, +0.0, −0.3 | 7.3 | 0.9 |
| [0, 0.10, 0] | sequential | yes | yes | −5.7, +2.8, +4.1 | 8.1 | 4.6 |
| [0, 0.10, 0] | mpc | yes | yes | −1.6, −0.0, −0.3 | 11.3 | 1.3 |
| [−0.10, −0.10, 0] | sequential | yes | yes | −2.6, −5.2, +0.9 | 7.9 | 1.5 |
| [−0.10, −0.10, 0] | mpc | yes | yes | −1.2, +0.0, −0.3 | 7.6 | 0.5 |
| [0.05, 0.08, 0.087] | sequential | yes | yes | −1.7, −1.0, −3.4 | 16.4 | 5.4 |
| [0.05, 0.08, 0.087] | mpc | yes | yes | −1.9, +0.0, −0.3 | 7.3 | 1.6 |
| [−0.08, −0.05, −0.087] | sequential | yes | yes | −2.0, +0.4, −7.1 | 9.3 | 0.9 |
| [−0.08, −0.05, −0.087] | mpc | yes | yes | −1.2, −0.0, −0.3 | 7.3 | 0.9 |

Other MPC runs from wrong starts, all aligned:

| Run | Start | TCP offset dx, dy, dz (mm) | Approach (s) |
|---|---|---|---|
| Repeat | [−0.10, 0, 0] | −1.4, +0.0, −0.3 (bottle after: 0.9 mm) | 7.3 |
| Video | [−0.08, 0.06, 0.07] | −1.4, +0.0, −0.3 | 7.4 |
| `test_wbmpc_pick_sim.py` | [−0.08, 0.06, 0.07] | −2.4, −0.0, −0.4 | 7.1 |
| `test_wbmpc_pick_sim.py` on the final code | [−0.08, 0.06, 0.07] | −3.5, +0.0, −0.4 | 7.1 |

### What the numbers say

**Both modes completed and were aligned on all twelve starts.** Neither mode
failed a case.

**The MPC places the gripper more consistently.**
- MPC: dx −1.2 to −3.5 mm, |dy| ≤ 0.05 mm, dz −0.3 to −0.4 mm. The TCP trails
  the moving target along the approach, as the planning spike predicted.
- Sequential: dx −1.7 to −6.0, dy −5.2 to +2.8, dz −7.1 to +5.7 mm. The base
  stops anywhere within 5 mm, and MoveIt's joint goals are met to 0.01 rad,
  which is several millimetres at the gripper.
- Two sequential runs came closest to a limit: dy −5.2 (limit ±6) and dz −7.1
  (limit ±10).
- The bottle ended 0.5–1.8 mm from its start after an MPC run and 0.9–5.4 mm
  after a sequential one.

**From an exact standoff, the scripted pick is faster:** 4.8 s against 7.1 s.
The MPC spends 5.3 s on the slide at 3 cm/s; the scripted base slides at
8 cm/s.

**Once the base starts elsewhere, the sequential pick pays for the
correction.**
- The corrective leg takes 2.4–4.6 s, for 7.2–16.4 s in total.
- In the 16.4 s run the corrective leg left the base 3 mm to the side. The
  small sideways correction during the slow slide-in then made the steering
  base stop to re-steer, so the slide-in took 10.1 s instead of about 3 s.

**The MPC pick takes 7.1–7.6 s from any of the starts, with one exception.**
The pure sideways start took 11.3 s: the pre-grasp took 6.1 s instead of about
2 s, because the base had to translate 10 cm sideways.

**The MPC keeps the base's yaw and the arm absorbs it.** At the attach the base
was still turned ±5° (+5.7° in one run: under translation-only commands the
heading drifted by 0.7°).

### Full transfer (house, no error)

- **Scripted:** completed; 125.2 s of wall time from the first step to
  "sequence complete". [sim_whole_body_mpc.md](sim_whole_body_mpc.md) measured
  125.6 s for the same run through the gate, and the recorded video is 124 s of
  sim time. Approach 4.9 s at (−5.3, −3.2, +3.5) mm.
- **MPC pick:** completed; 137.7 s. Approach 7.1 s at (−1.8, +0.0, −0.3) mm.
  The difference is the slower slide in, the slide-based lift and the two
  hand-overs.

### Running the comparison yourself

One fresh sim per run, because the transfer moves the object. Give every run
its own ROS domain and gz partition, and make sure no `ign gazebo` survives
the previous run.

In the first batch an `ign gazebo` server outlived its run (it ignored SIGINT
and SIGTERM). Every later run on the same domain then had a second
`/controller_manager`, and the gz partition held a second world. The result
was controller spawners that never loaded the arm controller, attach
confirmations that never came, and one MPC pre-grasp that never converged.
Those runs were discarded and the whole batch was repeated clean; that is the
table above.

```bash
i=0
for err in "[0.0, 0.0, 0.0]" "[-0.10, 0.0, 0.0]" "[0.0, 0.10, 0.0]" "[-0.10, -0.10, 0.0]" \
           "[0.05, 0.08, 0.087]" "[-0.08, -0.05, -0.087]"; do
  for mode in sequential mpc; do
    i=$((i + 1))
    export ROS_DOMAIN_ID=$((50 + i)) ROS_LOCALHOST_ONLY=1 IGN_PARTITION=trial_$i GZ_PARTITION=trial_$i
    setsid ros2 launch ranger_sim sim_bringup.launch.py headless:=true handover_gate:=true \
        launch_nav:=false > sim_$i.log 2>&1 &
    SIM=$!
    # wait until `ros2 control list_controllers` shows piper_arm_controller and
    # piper_gripper_controller active (and the stream controller inactive)
    ros2 launch ranger_sim pick_place_demo.launch.py execute:=true approach:=$mode \
        approach_only:=true approach_error:="$err" 2>&1 | grep "approach result"
    kill -INT -- -$SIM; sleep 10
    pgrep -af "ign gazebo"   # stop survivors: kill -KILL <pid>
  done
done
```

## Known limitations

- **The object's position comes from the scene.** Nothing reads the wrist
  camera.
- **No obstacle or self-collision constraints in the MPC.** The base box is
  checked against the table only. The house chair clearance (y ≥ −0.20) is
  reasoned, not checked, as in `config/wbmpc.yaml`.
- **Pick only.** The place stays scripted.
- **The REACHED latch makes the slide stop-and-go.** The MPC holds still
  whenever it is within 3 mm, so a target that crawls slower than it tracks is
  followed in small steps. At 3 cm/s it stayed in TRACKING for the whole slide
  in these runs.
- **Alignment is measured at the attach only.** The TCP's distance from the
  approach line during the slide is not logged; in the planning spike it was
  ≤ 1.7 mm.
- **Sim only.** The kinematic base does not slip. PARALLEL mode has never run
  on the hardware, and the solve times are from a workstation (median 8–11 ms
  here).
- **`approach_error` stands in for a Nav2 stop.** Nav2 is not part of this
  demo.
- **`launch_test` can leave an `ign gazebo` server behind** after the GPU
  tests (seen after `test_wbmpc_pick_sim.py`, `test_wbmpc_sim.py` and
  `test_sim_smoke.py`). Check `pgrep -af "ign gazebo"` before starting the next
  sim on the same domain.

## Key files

- `src/ranger_sim/scripts/wbmpc_pick_place_demo.py`: the MPC pick (`WbmpcPickPlaceDemo`, `pick_targets`)
- `src/ranger_sim/scripts/pick_place_demo.py`: `approach_error`, `approach_only`, the result line
  (`displaced_pose`, `tcp_offset_mm`, `is_aligned`, `format_approach_result`)
- `src/ranger_sim/scripts/wbmpc_reach_demo.py`: `reach_pose`, `slide`, `slide_point`
- `src/ranger_sim/launch/pick_place_nodes.py`: the demo and MPC nodes of both launches, the
  `approach*` arguments
- `src/ranger_sim/config/pick_place_scenes.yaml`: `grasp_tcp`, `hover_tcp`, `mpc_base_box`
- `src/ranger_sim/test/test_wbmpc_pick_place_demo.py`, `test_wbmpc_pick_sim.py` (GPU), and the
  additions to `test_pick_place_demo.py`, `test_wbmpc_reach_demo.py`, `test_wbmpc_launch.py`,
  `test_pick_place_geometry.py`, `test_pick_place_scenes.py`
