cmake_minimum_required(VERSION 3.5)
project(drone_demo_system_tests)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(src/system)

endif()

ament_package()
