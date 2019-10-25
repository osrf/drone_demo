cmake_minimum_required(VERSION 3.5)
project(sitl_launcher)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(drone_tf_broadcast
  src/drone_tf_broadcast.cpp
)

ament_target_dependencies(drone_tf_broadcast
  rclcpp
  px4_msgs
  tf2_ros
  geometry_msgs
)

install(PROGRAMS
  scripts/launch_drone_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
 DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  drone_tf_broadcast
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
