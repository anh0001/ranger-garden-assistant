# Handover to a seated person, Stage 0: rule baseline, hard limits, metrics (sim)

Experiment T0 of the forged idea
([rl-reference-governor-di-atas-mpc](forge/rl-reference-governor-di-atas-mpc/forged-idea.md)):
everything a learned "governor" above the whole-body MPC will need, except
the learning. A scripted person sits on the dining chair `ChairA_01_001` of the
house scene. A rule baseline picks the vitamin bottle from the table, chooses a
standoff next to the person from the person's **measured** state, drives
there, hands the base and the arm to the whole-body MPC
([sim_whole_body_mpc.md](sim_whole_body_mpc.md)), and either offers the bottle
to the extended hand or sets it down on the table. A separate node owns the
hard limits. A recorder writes one row of **ground-truth** metrics per
episode, and a batch runner produces the baseline table and the
faulty-person-data matrix below.

Gazebo Fortress only (`ranger_sim`): nothing here has a real-robot path.
`handover_safety` is a software interlock in a simulation; it is not a
safety-rated function, it has no redundancy, and nothing in it may be reused on
hardware as a safety measure.

Every threshold is a **starting value** from one lab, one study or engineering
judgement (`config/handover.yaml` names the source of each), not a validated
one.

## The episode

| Phase (`/handover_baseline/phase`) | What happens |
|---|---|
| `PICK` | the scripted pick of `pick_place_demo` (release, open, home, carry, drive to the table, look, slide in, attach, close, lift, carry) |
| `AWAY` | drive to `nav_away_pose` (−2, 0, 180°); the person turns to the episode's facing once the robot is 1.5 m away |
| `TRANSIT` | choose a standoff from the measured person, drive to its pre-standoff (0.30 m further out), wait there for the hand; re-choose if the facing turned more than 35° (published on every entry) |
| `APPROACH` | MPC base box = standoff ± 0.12 m, base to the MPC (gate), arm to streamed setpoints |
| `OFFER` / `PLACE` | MPC to the offer pose (hand extended) or, after 3 s without a hand, above the table place pose (TCP 0.962 m), straight down to it, and the place pose once more on the place topic; a standoff that cannot reach the table re-chooses with `need_place` first |
| `WAIT` | offer: the measured take, up to 10 s (then outcome `timeout`, the bottle stays in the gripper) |
| `RELEASE` | release permit from `handover_safety`, open, `/grasp/set` release |
| `RETREAT` | MPC back-off 0.15 m along the approach, hand back, carry pose, back to the pre-standoff, away |
| `DONE: <outcome>` / `FAILED: <step>` | outcomes `handover`, `set_down`, `timeout`, `no_standoff`, `stopped` (a step failed while `handover_safety` reported STOP), `failed` |

The person (`scripts/person_proxy.py`) is a script, not a human model: it faces
the table during the pick, turns when the robot has left, extends its hand
after a reaction delay once the robot has stood still within 1.5 m, takes the
object when the still TCP is within 0.65 m of its shoulder (after a second
delay), and then puts it on the table (one `set_pose` of the bottle). Its
models (`models/person_proxy.sdf`, `models/person_hand.sdf`) are visual only:
a contact is measured from ground truth, not simulated.

## What runs where

```
sim_bringup.launch.py handover_gate:=true
handover_baseline.launch.py execute:=true  (spawns person_proxy + person_hand)

person_proxy --/person/measured/{body_pose,hand_pose,hand_extended,take}--> handover_baseline, handover_safety
person_proxy --/person/ground_truth/*, /person/fault_active---------------> handover_recorder

handover_baseline --/handover_safety/scale_request (Float64, 10 Hz)--------\
handover_baseline --/handover_safety/{target_request,place_target_request}--> handover_safety
handover_baseline --/handover_safety/release (Trigger)----------------------/
handover_baseline --/whole_body_mpc/base_box (Float64MultiArray)--> whole_body_mpc

handover_safety --/whole_body_mpc/target (height cap checked)--> whole_body_mpc
handover_safety --/whole_body_mpc/speed_scale (Float64, 20 Hz)--> whole_body_mpc     (bounds of the plan)
handover_safety --/base_handover_gate/speed_scale (Float64)----> base_handover_gate (twist clamp, stop)
handover_safety --/speed_limit (nav2_msgs/SpeedLimit)----------> controller_server  (MPPI plans slower)
handover_safety --/base_handover_gate/set_governed (SetBool)---> base_handover_gate

scripted legs / Nav2 --/cmd_vel-------\
whole_body_mpc -------/cmd_vel_manip---> base_handover_gate --/cmd_vel_gated--> ranger_motion_adapter
```

`base_handover_gate` stays the only publisher into the motion adapter, and
`handover_safety` publishes no twist. The baseline has no publisher on
`/whole_body_mpc/target`, `/whole_body_mpc/speed_scale`,
`/base_handover_gate/speed_scale` or `/speed_limit`
(`test_handover_baseline.py` asserts it).

### The four decisions

`scripts/handover_core.py` defines the `Decisions` protocol; the baseline gets
one object and nothing else in the node decides. `RuleDecisions` is the rule
baseline; a learned policy will implement the same four methods:

| Method | Rule |
|---|---|
| `choose_standoff(person, candidates, need_place)` | among the statically free candidates that reach the offer pose (and the table, if needed): both reaches first, then the smallest absolute bearing, then the smallest distance |
| `request_scale(clearance_m)` | 0.3 at clearance ≤ 0.5 m, 1.0 at ≥ 1.5 m, linear between |
| `wait_or_go(waited_sec, hand_extended)` | offer as soon as the hand is extended, place after 3 s without it |
| `should_rechoose(facing_at_choice, facing_now, rechoices)` | the facing moved more than 35° and fewer than 2 re-choices so far |

Candidates are bearings {±90, ±60, ±35, 0}° from the person's facing × distances
{0.5 … 0.9} m from the torso axis to the robot's front edge, the robot facing
the person. A candidate is free when, at the standoff, the four corners of its
MPC box and ±0.10 rad, the footprint keeps 0.05 m to the person's capsule
(torso axis to knees, radius 0.22 m) and to the base obstacles, stays off the
table, and neither the shoulder → offer line nor the torso → offer line crosses
the backrest. The offer pose is 0.45 m from the torso axis towards the robot at
world z 0.80; the table place pose is the free table point (2 cm grid, 0.10 m
inside the edges, 0.06 m off the tableware) 0.35–0.55 m from the torso axis
that is nearest to the robot.

### The hard limits (`handover_safety`)

Every 50 ms: final scale = min(the requested scale if fresher than 0.5 s else
0; the limits), published to the MPC and to the gate, and as a Nav2
`SpeedLimit` (percentage, `max(scale, 0.05) × 100`; Nav2 reads 0.0 as "no
limit", so the stop itself is the gate's). The limits, in the order
`~/state` names them:

1. **TCP height cap** 1.00 m (odom): 0 while the TCP is above it, or unknown,
   or its transform is older than 0.5 s; a target above it is refused
   (`REJECTED: …` on `~/state`), not clamped.
2. **Watchdog**: 0 while the newest measured body pose is older than 0.3 s,
   by its stamp or by its receipt.
3. **Jump**: 0 for 1.0 s after the measured body moved more than 0.15 m
   between two samples.
4. **Stop distance**: 0 while the footprint is within 0.15 m of the measured
   capsule, and while `/odom` is missing or older than 0.5 s.

And 0 until `base_handover_gate` has confirmed it is governed
(`STOP: gate not governed`); the baseline does not start before that.

**Release permit** (`~/release`): only with a measured take in the last 1.0 s
while the measured person data is fresh and not jumping, or with the last
forwarded target from `~/place_target_request` and the MPC REACHED. A place
target must lie on the table top at the place height (± 5 cm), or it is
refused. The gripper and `/grasp/set` are not behind this node; the recorder's
`viol_release` audits the release against ground truth.

At start the node governs the gate (`~/set_governed true`, retried); on any
exit it publishes scale 0 once, removes the Nav2 limit (0.0) and ungoverns the
gate. A dead `handover_safety` that could not do that leaves the base stopped
until `ros2 service call /base_handover_gate/set_governed std_srvs/srv/SetBool "{data: false}"`
(the sim's gate only).

### The speed scale in the MPC and the gate

See [sim_whole_body_mpc.md](sim_whole_body_mpc.md): a `speed_scale` solver
parameter multiplies the base and joint speed bounds (identical inputs at
1.0, tested), the node pauses under 0.05 (`PAUSED`), and the gate's governed
mode clamps the twist. A node that never gets a scale behaves as before.

### Routes of the scripted base legs

The baseline drives every leg through `handover_core.route()`: the leg is
direct when the base keeps 0.35 m (`route_margin`) to the person's capsule and
0.05 m to the base obstacles all along it; otherwise it goes via one waypoint on
rings 1.2–2.4 m around the person (the shortest free route). The check follows
the path `pick_place_demo`'s pose controller really drives: a turn in place to
the leg's yaw, then **each body axis saturated on its own** at 0.25 m/s, which
with both errors large is a 45° dog-leg in the base frame, not the straight
line. In Gazebo the straight-line assumption put the base's corner 0.145 m from
the knees on the way to the pre-standoff (`STOP: stop distance`, outcome
`stopped`); `test_handover_core.py` pins that leg. With the routes, the base
backs straight out of the pick slot before it turns, and reaches the
pre-standoffs south of the table around the chair's west side.

## Running it

```bash
# Prerequisites: house assets fetched, casadi installed (README), workspace built.
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1   # never on a network with a robot

ros2 launch ranger_sim sim_bringup.launch.py headless:=true handover_gate:=true
# wait for the controllers (piper_arm_stream_controller inactive), then one episode:
ros2 launch ranger_sim handover_baseline.launch.py execute:=true
ros2 launch ranger_sim handover_baseline.launch.py execute:=true \
    facing_yaw:=0.5236 hand_side:=right hand_extends:=false \
    reaction_delay:="[2.0, 4.0]" results_file:=/tmp/episodes.csv
# A measured person with faults (counted from 1.2 m): a 2 s dropout 0.5 s in
ros2 launch ranger_sim handover_baseline.launch.py execute:=true \
    faults:="[0, 0, 0, 0.5, 2.0, 0, 0, 0, 0, 0, 0, 1.2]"

# Without execute:=true the baseline only plans the arm moves with MoveIt.
# One fresh sim per episode (the bottle moves): the batches do that for you.
python3 src/ranger_sim/tools/run_handover_batch.py --set baseline --dry-run
python3 src/ranger_sim/tools/run_handover_batch.py --set baseline --out ~/.ros/handover_results/baseline
python3 src/ranger_sim/tools/run_handover_batch.py --set faults --out ~/.ros/handover_results/faults

# Launch test: one episode in its own sim (GPU, house assets, casadi)
RANGER_SIM_GPU=1 launch_test src/ranger_sim/test/test_handover_sim.py
```

The results file defaults to `~/.ros/handover_results/handover_<time>.csv`
(nothing lands in the repository). The batch runner gives every episode its own
ROS domain (50 + index), `ROS_LOCALHOST_ONLY=1` and gz partition, refuses to
start while an `ign gazebo` runs, and stops each bringup by its process group
and then its session (Gazebo can leave the group). It starts only `ros2 launch`
and `ros2 control list_controllers`.

### Launch arguments (`handover_baseline.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `execute` | `false` | plan only (MoveIt plans, nothing is sent) |
| `use_nav` | `false` | `true`: the legs as Nav2 goals (see Known limitations) |
| `scene`, `world_name` | `small_house`, scene's | the house: the chair |
| `person_x`, `person_y` | 0.504, −0.858 | torso axis (`ChairA_01_001`) |
| `facing_yaw` | 0.0 | the facing (rad) after the turn |
| `hand_side`, `hand_extends` | `left`, `true` | `hand_extends:=false`: the set-down case |
| `reaction_delay` | `[0.5, 1.0]` | s, both delays drawn uniformly from it |
| `facing_change` | 0.0 | rad added 1 s after the robot stood still nearby |
| `faults` | 12 zeros | `[bias_x, bias_y, noise_std, dropout_start, dropout_sec, stale_start, stale_sec, latency_sec, jump_at, jump_x, jump_y, trigger_distance]` |
| `seed`, `episode_id`, `results_file` | 0, 0, empty | copied into the row |

## Starting values (`config/handover.yaml`)

The file is the source; the main values and where they come from (references
of [the research file](research/academic-lit-human-friendly-robot-approach-and-handov-2026-11-25/research.md),
"Starting specification"):

| Value | Starting value | Source |
|---|---|---|
| offer height, offer distance | world z 0.80 m, 0.45 m | [2] |
| standoff distances | 0.5–0.9 m (torso axis → front edge) | 0.5–0.7 m [1][2]; 0.8, 0.9 added |
| bearings, view limits | ±35° comfortable, ±70° uncomfortable | [6] |
| stop distance (hard) | 0.15 m | engineering; sized next by the bias sweep below |
| TCP height cap (hard) | 1.00 m odom | engineering: 5 cm above the carry pose |
| speed ramp | 0.3 at ≤ 0.5 m, 1.0 at ≥ 1.5 m | "lower as distance shrinks" [14][9]; 0.3 × 0.26 m/s = 0.078 m/s, low end of [16] |
| watchdog, jump | 0.3 s, 0.15 m, hold 1.0 s | engineering |
| waits | hand 3 s, take 10 s | "bounded waiting", engineering |
| re-choice | 35°, at most 2 | [6]; engineering |
| reach proxy | offer 0.58 m, table 0.48 m (shoulder, plan view) | the reach check below |
| route margin | 0.35 m | engineering: stop distance + 0.20 m |
| base obstacles | chair, refrigerator, south wall | collision meshes; lidar scan (2026-11-25) |

## Reach check (`test/test_handover_reach.py`, 2026-11-25)

The MPC core on a kinematic plant (default `MpcConfig`, posture CARRY), each
case from the standoff and from the box corner nearest the pre-standoff with
the yaw 0.10 rad off: 45 cases pass in 64 s. The offer pose is reached from all
pinned cases (facing 0°: (90°, 0.5 m), (90°, 0.7 m), (−60°, 0.7 m), (−90°, 0.6 m);
60°: (35°, 0.8 m); 90°: (0°, 0.9 m), (35°, 0.9 m)); the table place pose from
facing 0° (90°, 0.5 m), (−60°, 0.7 m), (−60°, 0.8 m) and 30° (−90°, 0.7 m), and
not from facing 0° (90°, 0.8 m) or 90° (0°, 0.9 m) (stays > 20 mm off). Every
candidate the proxy flags for facings 0, 30, 60 and 90° is reached (no false
positive, 32 cases).

What the rule then chooses (`test_handover_core.py`):

| Facing | Free candidates | Reach both offer and table | Rule's choice |
|---|---|---|---|
| 0° | 13 | (−60°, 0.7 m) only | (−60°, 0.7 m) |
| 30° | 10 | (−90°, 0.7 m) only | (−90°, 0.7 m) |
| 45° | 3 | none | (60°, 0.7 m) |
| 60° | 5 | none | (35°, 0.8 m) |
| 90° | 2 | none | (0°, 0.9 m) |

From 45° on no candidate reaches the table: a person facing away from it with
no hand extended ends `no_standoff` after the `need_place` re-choice. In front
of a person facing the table there is the table: the only standoff that reaches
both poses is at −60°, outside the comfortable ±35°.

## Gazebo checks (2026-11-25, house, RTX 6000 Ada workstation)

1. **Chair facing**: the chair's collision and visual meshes put the backrest
   (every vertex above 0.7 m) at world x 0.271–0.340, on the −x side of the torso
   axis (`test_handover_core.py` pins it); the lidar at 0.63 m sees it there. The
   person faces +x, the table.
2. **`set_pose` on static models**: Gazebo reported `person_proxy` at yaw
   0.5236 rad after the turn of a 30° episode, and the bottle at
   (1.000, −0.620, 0.801) after the take. No non-static fallback needed.
3. **`/speed_limit`**: the controller server subscribes RELIABLE / VOLATILE; the
   safety node's RELIABLE / TRANSIENT_LOCAL publisher matches.
4. **Floor around the standoffs**: a lidar scan from (−0.20, −0.50) showed the
   refrigerator (`Refrigerator_01_001`, face at y −2.02) and the south wall
   (y ≈ −3.05) south of the chair; both are now base obstacles. The standoffs
   and their pre-standoffs keep 0.05 m to them.
5. **One episode, default person** (facing the table, left hand, delay
   0.5–1.0 s, scripted legs): `DONE: handover`, success, no violation. Standoff
   (−60°, 0.7 m); offer REACHED at 4.0 mm after 135 solves (median 18 ms, base
   moved 0.19 m); take after 2.6 s of WAIT; release permitted on the take;
   ground-truth minimum clearance 0.375 m; 121 s from TRANSIT to DONE.
   `/whole_body_mpc/state` never showed FAULT.
6. **Set-down** (facing the table, no hand): reaching for the place pose
   directly stalled 115 mm short for 40 s: the MPC lowers the held bottle while
   it reaches across the table's south edge, and the bottle meets the edge. Now
   the MPC goes above the place point first (TCP 0.962 m, the scene's hover
   height), slides straight down at 0.03 m/s and sends the place pose once more
   on the place topic: `DONE: set_down`, success, permit "set down at the table
   place pose". At the place pose the MPC settled 9.1–9.7 mm off (4.3–5.0 mm on
   the kinematic plant): with the scale at 0.3 the base command decays under the
   0.01 m/s base deadband and the base parks first. The handover launch runs the
   MPC with a 15 mm REACHED tolerance (5 mm in `config/wbmpc.yaml`).

## Baseline (2026-11-26, house, 34 episodes, seed 0)

Command: `python3 src/ranger_sim/tools/run_handover_batch.py --set baseline --out ~/.ros/handover_results/baseline --seed 0`
(scripted legs; code: the working tree on top of `72e9017`). Every number below is from that
`episodes.csv`. Outcomes: failed 4, handover 7, no_standoff 12, set_down 6, stopped 2, timeout 3; success 13 of 34.
Violations: contact in 0 episodes, stop distance while moving in 2 (#32, #33),
height cap in 0, release without take or place in 0.
Over the 13 delivered episodes: minimum ground-truth clearance median 0.301 (min 0.234, max 0.381) m;
time outside ±35° within 1.5 m median 42.744 (min 0.000, max 47.228) s; closing speed median 0.139 (min 0.037, max 0.150) m/s.

| # | facing | hand | reaches | delay s | outcome | ok | re-ch. | min clear. m | out 35° s | out 70° s | closing m/s | reach m | wait s | dur. s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 0° | left | yes | 0.5–1 | timeout | 0 | 0 | 0.379 | 42.7 | 37.1 | 0.137 | – | 10.0 | 123 |
| 1 | 0° | left | yes | 2–4 | timeout | 0 | 0 | 0.378 | 42.7 | 37.1 | 0.139 | – | 10.0 | 122 |
| 2 | 0° | left | no | 0.5–1 | set_down | 1 | 0 | 0.281 | 46.0 | 37.5 | 0.140 | – | 0.0 | 128 |
| 3 | 0° | left | no | 2–4 | failed | 0 | 0 | – | 0.0 | 0.0 | 0.000 | – | 0.0 | – |
| 4 | 0° | right | yes | 0.5–1 | handover | 1 | 0 | 0.376 | 42.6 | 37.1 | 0.150 | 0.27 | 1.6 | 117 |
| 5 | 0° | right | yes | 2–4 | handover | 1 | 0 | 0.381 | 42.7 | 37.2 | 0.139 | 0.27 | 4.6 | 116 |
| 6 | 0° | right | no | 0.5–1 | set_down | 1 | 0 | 0.272 | 47.2 | 37.5 | 0.139 | – | 0.0 | 125 |
| 7 | 0° | right | no | 2–4 | set_down | 1 | 0 | 0.279 | 46.4 | 38.0 | 0.139 | – | 0.0 | 124 |
| 8 | 30° | left | yes | 0.5–1 | failed | 0 | 0 | – | 0.0 | 0.0 | 0.000 | – | 0.0 | – |
| 9 | 30° | left | yes | 2–4 | timeout | 0 | 0 | 0.381 | 43.0 | 43.0 | 0.140 | – | 10.0 | 119 |
| 10 | 30° | left | no | 0.5–1 | set_down | 1 | 0 | 0.381 | 46.3 | 46.3 | 0.135 | – | 0.0 | 121 |
| 11 | 30° | left | no | 2–4 | set_down | 1 | 0 | 0.381 | 45.4 | 45.4 | 0.138 | – | 0.0 | 119 |
| 12 | 30° | right | yes | 0.5–1 | failed | 0 | 0 | – | 0.0 | 0.0 | 0.000 | – | 0.0 | – |
| 13 | 30° | right | yes | 2–4 | handover | 1 | 0 | 0.381 | 41.9 | 41.9 | 0.138 | 0.41 | 4.9 | 113 |
| 14 | 30° | right | no | 0.5–1 | failed | 0 | 0 | – | 0.0 | 0.0 | 0.000 | – | 0.0 | – |
| 15 | 30° | right | no | 2–4 | set_down | 1 | 0 | 0.380 | 46.3 | 46.3 | 0.138 | – | 0.0 | 122 |
| 16 | 60° | left | yes | 0.5–1 | handover | 1 | 0 | 0.274 | 5.3 | 0.0 | 0.063 | 0.23 | 1.4 | 74 |
| 17 | 60° | left | yes | 2–4 | no_standoff | 0 | 1 | 0.510 | 6.0 | 0.0 | 0.000 | – | 0.0 | 26 |
| 18 | 60° | left | no | 0.5–1 | no_standoff | 0 | 1 | 0.511 | 6.6 | 0.0 | 0.000 | – | 0.0 | 25 |
| 19 | 60° | left | no | 2–4 | no_standoff | 0 | 1 | 0.511 | 6.8 | 0.0 | 0.000 | – | 0.0 | 25 |
| 20 | 60° | right | yes | 0.5–1 | handover | 1 | 0 | 0.301 | 5.5 | 0.0 | 0.150 | 0.48 | 2.3 | 77 |
| 21 | 60° | right | yes | 2–4 | no_standoff | 0 | 1 | 0.510 | 6.4 | 0.0 | 0.000 | – | 0.0 | 25 |
| 22 | 60° | right | no | 0.5–1 | no_standoff | 0 | 1 | 0.507 | 7.2 | 0.0 | 0.000 | – | 0.0 | 26 |
| 23 | 60° | right | no | 2–4 | no_standoff | 0 | 1 | 0.511 | 6.7 | 0.0 | 0.000 | – | 0.0 | 25 |
| 24 | 90° | left | yes | 0.5–1 | handover | 1 | 0 | 0.234 | 0.0 | 0.0 | 0.037 | 0.32 | 1.9 | 86 |
| 25 | 90° | left | yes | 2–4 | no_standoff | 0 | 1 | 0.528 | 0.0 | 0.0 | 0.000 | – | 0.0 | 30 |
| 26 | 90° | left | no | 0.5–1 | no_standoff | 0 | 1 | 0.528 | 0.0 | 0.0 | 0.000 | – | 0.0 | 35 |
| 27 | 90° | left | no | 2–4 | no_standoff | 0 | 1 | 0.528 | 0.0 | 0.0 | 0.000 | – | 0.0 | 28 |
| 28 | 90° | right | yes | 0.5–1 | handover | 1 | 0 | 0.237 | 0.0 | 0.0 | 0.150 | 0.32 | 1.5 | 85 |
| 29 | 90° | right | yes | 2–4 | no_standoff | 0 | 1 | 0.529 | 0.0 | 0.0 | 0.000 | – | 0.0 | 27 |
| 30 | 90° | right | no | 0.5–1 | no_standoff | 0 | 1 | 0.528 | 0.0 | 0.0 | 0.000 | – | 0.0 | 28 |
| 31 | 90° | right | no | 2–4 | no_standoff | 0 | 1 | 0.528 | 0.0 | 0.0 | 0.000 | – | 0.0 | 27 |
| 32 | 0° +46° | left | yes | 0.5–1 | stopped | 0 | 1 | 0.146 | 132.1 | 132.1 | 0.139 | – | 0.0 | 141 |
| 33 | 30° +46° | right | yes | 0.5–1 | stopped | 0 | 1 | 0.146 | 131.6 | 131.6 | 0.140 | – | 0.0 | 140 |

What the table says:
- **The rule ignores the hand side.** Facing 0° and 30° the only standoff that reaches both poses is on
  the person's right, so a left hand reaches across the body: the offer is 0.644 m and 0.661 m from the left
  shoulder against the script's 0.65 m reach, and those episodes end `timeout` (#0, 1, 9). The right hand takes
  (#4, 5, 13).
- **Facing away from the table (60°, 90°) there is no set-down:** no standoff reaches the table, so a person
  who does not reach (no hand, or a hand slower than 3 s: delay 2–4 s) ends `no_standoff` after the
  `need_place` re-choice; with the fast hand the offer succeeds (#16, 20, 24, 28).
- **Comfort is not what the rule optimises:** the standoffs at 0°/30° are at −60°/−90° bearing, so the base
  spends tens of seconds outside ±35° (and ±70°) within 1.5 m of the person.
- **Facing change (#32, 33):** after the person turned 46° the re-choice had to cross from the south to the
  north side of the chair; `route()` found no one-waypoint route there and drove direct, and `handover_safety`
  stopped the base 0.146 m from the person (0.1 s of motion under 0.15 m: the stop latency). A route with two
  waypoints is missing.
- **#3, 8, 12, 14 failed at the pick's attach** (the sim's 1 s confirmation, after the baseline's retry): not
  handover results; see the re-run rows below.
- The MPC base box and the 15 mm tolerance held: no `FAULT`, no height-cap violation.

### Re-run of the four attach failures (same seeds)

`run_handover_batch.py --set baseline --only 3 8 12 14 --out ~/.ros/handover_results/baseline_rerun --seed 0`; both rows of each episode are kept in their files.

| # | outcome | ok | min clear. m | out 35° s | reach m | wait s |
|---|---|---|---|---|---|---|
| 3 | set_down | 1 | 0.279 | 45.5 | – | 0.0 |
| 8 | timeout | 0 | 0.381 | 42.0 | – | 10.0 |
| 12 | handover | 1 | 0.379 | 42.4 | 0.41 | 2.4 |
| 14 | set_down | 1 | 0.380 | 43.6 | – | 0.0 |

With the re-runs the 34 cells give 8 handovers, 8 set-downs, 4 timeouts (all left hand at 0°/30°), 12 no_standoff, 2 stopped, 0 failed; one bringup of the re-run (#12) was rejected for objects left on the gripper and retried.


## Faulty person data (2026-11-26, 27 episodes)

`run_handover_batch.py --set faults --out ~/.ros/handover_results/faults --seed 0`: facing 0°, left hand, delay 0.5–1.0 s, scripted legs. The injector arms once the robot has been 1.2 m from the torso axis; window faults start 0.5 s after the base next comes closer than that; bias applies from arming. Ground truth is unchanged. A first run (archived as invalid) armed at startup, so its faults hit the pick.

| # | fault | outcome | stops | latency s | motion under fault s | stop-dist. s | contact s | min clear. m |
|---|---|---|---|---|---|---|---|---|
| 0 | dropout 0.5 s | failed | 0 | – | – | 0.00 | 0.00 | – |
| 1 | dropout 2 s | failed | 0 | – | – | 0.00 | 0.00 | – |
| 2 | dropout to the end | failed | 0 | – | – | 0.00 | 0.00 | – |
| 3 | stale 2 s | timeout | 1 | 0.40 | 0.0 | 0.00 | 0.00 | 0.379 |
| 4 | stale to the end | stopped | 1 | 0.40 | 0.0 | 0.00 | 0.00 | 0.517 |
| 5 | jump 0.3 m toward | stopped | 2 | 0.15 | 50.0 | 0.00 | 0.00 | 0.380 |
| 6 | jump 0.3 m away | handover | 1 | 0.20 | 88.8 | 3.25 | 0.00 | 0.138 |
| 7 | jump 1 m toward | stopped | 2 | 0.15 | 20.4 | 0.00 | 0.00 | 0.380 |
| 8 | jump 1 m away | failed | 1 | 0.15 | 28.3 | 0.00 | 0.00 | 0.229 |
| 9 | bias 0.05 m away #0 | handover | 0 | – | – | 0.00 | 0.00 | 0.335 |
| 10 | bias 0.05 m away #1 | handover | 0 | – | – | 0.00 | 0.00 | 0.339 |
| 11 | bias 0.05 m away #2 | handover | 0 | – | – | 0.00 | 0.00 | 0.340 |
| 12 | bias 0.1 m away #0 | handover | 0 | – | – | 0.00 | 0.00 | 0.296 |
| 13 | bias 0.1 m away #1 | bringup_failed | 0 | – | – | 0.00 | 0.00 | – |
| 14 | bias 0.1 m away #2 | handover | 0 | – | – | 0.00 | 0.00 | 0.296 |
| 15 | bias 0.15 m away #0 | handover | 0 | – | – | 0.00 | 0.00 | 0.323 |
| 16 | bias 0.15 m away #1 | handover | 0 | – | – | 0.00 | 0.00 | 0.323 |
| 17 | bias 0.15 m away #2 | handover | 0 | – | – | 0.00 | 0.00 | 0.323 |
| 18 | bias 0.2 m away #0 | handover | 0 | – | – | 0.00 | 0.00 | 0.185 |
| 19 | bias 0.2 m away #1 | handover | 0 | – | – | 0.00 | 0.00 | 0.185 |
| 20 | bias 0.2 m away #2 | handover | 0 | – | – | 0.00 | 0.00 | 0.184 |
| 21 | bias 0.3 m away #0 | handover | 0 | – | – | 0.00 | 0.00 | 0.707 |
| 22 | bias 0.3 m away #1 | handover | 0 | – | – | 0.00 | 0.00 | 0.714 |
| 23 | bias 0.3 m away #2 | handover | 0 | – | – | 0.00 | 0.00 | 0.731 |
| 24 | bias 0.4 m away #0 | timeout | 0 | – | – | 0.00 | 0.00 | 0.804 |
| 25 | bias 0.4 m away #1 | timeout | 0 | – | – | 0.00 | 0.00 | 0.803 |
| 26 | bias 0.4 m away #2 | timeout | 0 | – | – | 0.00 | 0.00 | 0.806 |

Against the rule stated before the runs (stop within 0.5 s of sim time, no motion while the fault lasts, no stop-distance or contact violation after the fault edge):
- **Stale data: pass** (#3, #4): stopped within 0.4 s, no motion under the fault, no violation.
- **Jumps: the stop passes, the episode does not.** Each jump stopped the base within 0.15–0.20 s, for the 1 s hold. After the hold the shifted position is accepted: the base moves on (20–89 s of motion with the offset still there), and with a 0.3 m jump away from the robot (#6) it spent 3.2 s inside the stop distance (ground-truth clearance 0.138 m). The hard limits trust a persistent offset; the watchdog only sees the edge. (In the invalid first run a 0.3 m jump led to contact: clearance −0.021 m.)
- **Dropout: not tested.** All three episodes failed in the drive away before the fault was active (no fault edge, no safety stop); the cause of that leg failure is open.
- **Bias (measurement, away from the robot):** 0.05–0.30 m all handed over without a violation; the smallest ground-truth clearance was 0.334–0.340 m (0.05 m), 0.296 m (0.10 m), 0.323 m (0.15 m), 0.184–0.185 m (0.20 m), 0.707–0.731 m (0.30 m, the choice moved to another side); at 0.40 m every episode timed out (the offer moved out of the person's reach). No bias up to 0.40 m produced a violation, so this sweep gives no evidence to change `stop_distance`; it shows the clearance shrinking by about the bias up to 0.20 m. #13's bringup failed twice.

## Known limitations

- **Nav2 legs** (`use_nav:=true`): in Gazebo Nav2 (RotationShim + MPPI, the
  steering base) did not reach the route's first goal, a straight 0.88 m
  reverse out of the pick slot, in 120 s (`follow_path` aborted repeatedly; the
  base moved 9 cm). The tables use the scripted legs; Nav2 parameters are out
  of scope here. Nav2's path is not checked by `route()`.
- The person is a script: one chair, fixed reaction rules, no head, no gaze, no
  arm model. The "measured" person is the script through a fault injector, not
  a perception pipeline.
- No collision geometry on the person and no collision constraints in the MPC:
  the arm may pass through the person, the chair or the tableware; only the
  candidate filter (backrest lines) and the offer height keep it away. The
  measured TCP path is not checked.
- The height cap does not stop a MoveIt arm motion (the trajectory controller is
  not behind `handover_safety`), and the release permit is not an interlock on
  the gripper: both are checked by the decision maker and audited by the
  recorder.
- A pure rotation of the base is not slowed by the scale (only stopped at 0).
- Near the reach limit at scale 0.3 the MPC's base parks under its 0.01 m/s
  deadband before the TCP is within 5 mm (see Gazebo check 6); the handover
  MPC therefore reaches to 15 mm. The MPC's cost and deadband are unchanged.
- The baseline sends the MPC's base box itself (`/whole_body_mpc/base_box`); the
  safety node does not check it. The box only bounds where the MPC may move the
  base; the stop distance still applies everywhere.
- The pick prefix carries the sim's own flakes, outside the handover: the
  attach depends on `grasp_manager`'s 1 s wall-clock confirmation (the baseline
  retries it once), and at some bringups `grasp_manager` cannot detach every
  graspable object at startup ("may stay welded to the gripper"), after which
  the scripted pick strikes the bottle (TCP 12–14 cm off at the attach). The
  batch runner now rejects such a bringup and retries it; the counts are in the
  tables below.
- The MoveIt move from 'home' to the carry pose lifts the TCP above 1.00 m for
  about a second: `handover_safety` reports `STOP: tcp above the height cap`
  during the pick. The base is still then, so nothing is lost, but the cap is
  crossed by a motion it does not govern.
- Base obstacles are hand-measured rectangles (meshes, a lidar scan), not a
  costmap. Other furniture of the house is not in the config.
- All thresholds are starting values; the reach proxy's thresholds come from a
  kinematic plant, not from Gazebo.

## Key files

| File | What |
|---|---|
| `src/ranger_sim/scripts/handover_core.py` | geometry, candidates, poses, ramp, routes, `Decisions` / `RuleDecisions`, config |
| `src/ranger_sim/scripts/person_proxy.py` | `PersonScript`, `FaultInjector`, the person node |
| `src/ranger_sim/scripts/handover_safety.py` | `SafetyRules`, the hard limits node |
| `src/ranger_sim/scripts/handover_baseline.py` | the rule baseline (a `WbmpcReachDemo`) |
| `src/ranger_sim/scripts/handover_recorder.py` | `EpisodeMetrics`, the CSV row |
| `src/ranger_sim/launch/handover_baseline.launch.py`, `handover_common.py` | the launch |
| `src/ranger_sim/config/handover.yaml` | every threshold with its source |
| `src/ranger_sim/tools/run_handover_batch.py` | the episode sets |
| `src/ranger_sim/test/test_handover_*.py`, `test_person_proxy.py`, `test_run_handover_batch.py` | tests (`test_handover_sim.py`: Gazebo, GPU) |
