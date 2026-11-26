cmake_minimum_required(VERSION 3.8)
project(ranger_sim)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Simulator-agnostic grasp API (/grasp/set), served by grasp_manager.py
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGrasp.srv"
)
ament_export_dependencies(rosidl_default_runtime)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python node scripts as executables
install(PROGRAMS
  scripts/base_handover_gate.py
  scripts/base_joint_state_publisher.py
  scripts/pick_place_demo.py
  scripts/grasp_manager.py
  scripts/handover_baseline.py
  scripts/handover_recorder.py
  scripts/handover_safety.py
  scripts/mimic_joint_state_relay.py
  scripts/person_proxy.py
  scripts/ranger_motion_adapter.py
  scripts/record_demo_video.py
  scripts/wbmpc_pick_place_demo.py
  scripts/wbmpc_reach_demo.py
  scripts/whole_body_mpc.py
  DESTINATION lib/${PROJECT_NAME}
)
# Loaded by path by the nodes next to them (no rclpy); not nodes.
# whole_body_mpc_core.py: casadi + numpy, for whole_body_mpc.py.
# handover_core.py: geometry and rules of the handover baseline.
install(FILES
  scripts/handover_core.py
  scripts/whole_body_mpc_core.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF/xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install models
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install test files
if(BUILD_TESTING)
  # No source file carries a per-file copyright header (the package license
  # is declared in package.xml), so skip ament_copyright as the
  # `ros2 pkg create` template does.
  set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Tests publish on /grasp/* and spawn a simulator: keep their traffic off
  # any real robot or running sim (local-only DDS, own domain and gz
  # partition).
  set(RANGER_SIM_TEST_ENV
    ROS_LOCALHOST_ONLY=1
    ROS_DOMAIN_ID=99
    IGN_PARTITION=ranger_sim_test
    GZ_PARTITION=ranger_sim_test
  )

  # Pure-logic unit tests (no simulator).
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_grasp_manager test/test_grasp_manager.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_mimic_joint_state_relay test/test_mimic_joint_state_relay.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_ranger_motion_adapter test/test_ranger_motion_adapter.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_record_demo_video test/test_record_demo_video.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_video_rig_launch test/test_video_rig_launch.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_pick_place_geometry test/test_pick_place_geometry.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_pick_place_demo test/test_pick_place_demo.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_pick_place_scenes test/test_pick_place_scenes.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_generate_small_house_world test/test_generate_small_house_world.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_fetch_sim_assets test/test_fetch_sim_assets.py
    ENV ${RANGER_SIM_TEST_ENV})
  # Whole-body MPC handover (docs/sim_whole_body_mpc.md).
  ament_add_pytest_test(test_base_handover_gate test/test_base_handover_gate.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_wbmpc_launch test/test_wbmpc_launch.py
    ENV ${RANGER_SIM_TEST_ENV})
  # Skipped without casadi (pip3 install --user casadi).
  ament_add_pytest_test(test_whole_body_mpc_core test/test_whole_body_mpc_core.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_whole_body_mpc test/test_whole_body_mpc.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_wbmpc_reach_demo test/test_wbmpc_reach_demo.py
    ENV ${RANGER_SIM_TEST_ENV})
  # Whole-body MPC pick in the pick-and-place demo (docs/sim_wbmpc_pick.md).
  ament_add_pytest_test(test_wbmpc_pick_place_demo test/test_wbmpc_pick_place_demo.py
    ENV ${RANGER_SIM_TEST_ENV})
  # Handover baseline, Stage 0 (docs/sim_handover_baseline.md).
  ament_add_pytest_test(test_handover_core test/test_handover_core.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_person_proxy test/test_person_proxy.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_handover_safety test/test_handover_safety.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_handover_baseline test/test_handover_baseline.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_handover_recorder test/test_handover_recorder.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_handover_launch test/test_handover_launch.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_run_handover_batch test/test_run_handover_batch.py
    ENV ${RANGER_SIM_TEST_ENV})
  # Reach check of the standoffs against the MPC: skipped without casadi; ~1 min.
  ament_add_pytest_test(test_handover_reach test/test_handover_reach.py
    TIMEOUT 600 ENV ${RANGER_SIM_TEST_ENV})

  # Headless Gazebo bringup. Needs a GPU (gpu_lidar / camera sensors), so it
  # is skipped unless RANGER_SIM_GPU=1 is set in the environment.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_smoke.py TIMEOUT 240 ENV ${RANGER_SIM_TEST_ENV})
  # Whole-body MPC reach in Gazebo (also needs casadi); RANGER_SIM_GPU=1 as above.
  add_launch_test(test/test_wbmpc_sim.py TIMEOUT 400 ENV ${RANGER_SIM_TEST_ENV})
  # Whole-body MPC pick from a wrong standoff (needs the house assets too).
  add_launch_test(test/test_wbmpc_pick_sim.py TIMEOUT 400 ENV ${RANGER_SIM_TEST_ENV})
  # One handover baseline episode in the house (GPU, casadi, house assets).
  add_launch_test(test/test_handover_sim.py TIMEOUT 600 ENV ${RANGER_SIM_TEST_ENV})

  install(DIRECTORY test/
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_package()
