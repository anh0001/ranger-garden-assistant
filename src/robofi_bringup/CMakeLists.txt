cmake_minimum_required(VERSION 3.8)
project(robofi_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/fastlio2_auto_relocalize.py
  scripts/camera_info_frame_id_fixer.py
  scripts/servo_array_to_piper_joint_cmd.py
  scripts/servo_singularity_recovery_supervisor.py
  scripts/piper_joint_state_sanitizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
