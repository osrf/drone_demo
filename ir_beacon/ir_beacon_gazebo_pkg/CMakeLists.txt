cmake_minimum_required(VERSION 2.5)
project(ir_beacon_gazebo_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(ir_beacon_msgs REQUIRED)

add_library(gazebo_irsensor_plugin SHARED
  plugins/IRSensorPlugin.cc
)

ament_target_dependencies(gazebo_irsensor_plugin
  gazebo_dev
  gazebo_ros
  ir_beacon_msgs
)

install(DIRECTORY
  worlds
  models
 DESTINATION share/${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

target_link_libraries(gazebo_irsensor_plugin ${GAZEBO_libraries})
