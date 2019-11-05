cmake_minimum_required(VERSION 3.5)

project(drone_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(proposed_aerial_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/drone_node_main.cpp
  src/drone_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "proposed_aerial_msgs"
  "px4_msgs"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

install(  TARGETS
            ${PROJECT_NAME}
          DESTINATION
            lib/${PROJECT_NAME}
)

ament_package()
