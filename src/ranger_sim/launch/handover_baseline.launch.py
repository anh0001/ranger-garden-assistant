#!/usr/bin/env python3
r"""
Run one episode of the handover rule baseline against an already-running sim.

Prereq: the house sim with the base handover gate (and Nav2 only for
use_nav:=true, which did not reach the routed goals in Gazebo; see the doc):

  ros2 launch ranger_sim sim_bringup.launch.py handover_gate:=true

Safe, plan the arm moves only (no motion, no hand-over, nothing published
by the baseline):
  ros2 launch ranger_sim handover_baseline.launch.py

Run one episode in Gazebo (moves the base and the arm), e.g. a person
turned 30 deg, reaching with the right hand:
  ros2 launch ranger_sim handover_baseline.launch.py execute:=true
  ros2 launch ranger_sim handover_baseline.launch.py execute:=true \
      facing_yaw:=0.5236 hand_side:=right results_file:=/tmp/episodes.csv

Spawns the seated person and its hand on ChairA_01_001 and starts
person_proxy, handover_safety, whole_body_mpc, handover_baseline and
handover_recorder (scripts/*.py). The episode: the scripted pick, away,
choose a standoff from the measured person, transit, wait for the hand,
hand-over to the whole-body MPC, offer or set down, release, retreat. The
recorder appends one ground-truth row to results_file. The launch ends
with the baseline (or with handover_safety). Parameters:
config/handover.yaml and config/wbmpc.yaml. Simulation only. See
docs/sim_handover_baseline.md.
"""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

_spec = importlib.util.spec_from_file_location(
    "handover_common", Path(__file__).with_name("handover_common.py"))
common = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(common)


def generate_launch_description():
    scene = LaunchConfiguration("scene")
    nodes = common.handover_nodes(scene, LaunchConfiguration("use_sim_time"))
    return LaunchDescription(
        common.handover_arguments() + common.person_spawns(scene) + nodes
        + common.end_with_the_baseline(nodes)
    )
