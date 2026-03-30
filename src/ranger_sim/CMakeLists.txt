cmake_minimum_required(VERSION 3.8)
project(ranger_sim)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  install(DIRECTORY test/
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_package()
