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
  scripts/base_joint_state_publisher.py
  scripts/pick_place_demo.py
  scripts/grasp_manager.py
  scripts/mimic_joint_state_relay.py
  scripts/record_demo_video.py
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
  ament_add_pytest_test(test_record_demo_video test/test_record_demo_video.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_video_rig_launch test/test_video_rig_launch.py
    ENV ${RANGER_SIM_TEST_ENV})
  ament_add_pytest_test(test_pick_place_geometry test/test_pick_place_geometry.py
    ENV ${RANGER_SIM_TEST_ENV})

  # Headless Gazebo bringup. Needs a GPU (gpu_lidar / camera sensors), so it
  # is skipped unless RANGER_SIM_GPU=1 is set in the environment.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_smoke.py TIMEOUT 240 ENV ${RANGER_SIM_TEST_ENV})

  install(DIRECTORY test/
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_package()
