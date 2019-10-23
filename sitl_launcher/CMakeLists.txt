cmake_minimum_required(VERSION 3.5)
project(sitl_launcher)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/launch_drone_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
)

ament_package()
