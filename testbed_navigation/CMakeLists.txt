cmake_minimum_required(VERSION 3.8)
project(testbed_navigation)

# This package contains no compiled code: only launch files, parameter files,
# a behavior tree, an RViz configuration and a Python helper script.
find_package(ament_cmake REQUIRED)

# Everything that launch files look up with get_package_share_directory()/FindPackageShare
# must be installed into share/testbed_navigation, otherwise it only exists in src/.
install(
  DIRECTORY
    launch
    config
    behavior_trees
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Executable helper scripts go to lib/<package> so that `ros2 run` can find them.
install(
  PROGRAMS
    scripts/send_goals.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
