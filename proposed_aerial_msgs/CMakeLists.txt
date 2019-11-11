cmake_minimum_required(VERSION 3.5)
project(proposed_aerial_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


set(msg_files
  msg/AngularVelocity.msg
  msg/Attitude.msg
  msg/FlightMode.msg
  msg/GlobalPosition.msg
  msg/GlobalWaypoints.msg
  msg/LinearAcceleration.msg
  msg/MavlinkAnnotatedWaypoints.msg
  msg/MavlinkWaypointParameters.msg
  msg/OdometryWithAcceleration.msg
  msg/UpperTriangularCovariance.msg
  msg/UpperTriangularCovariance3.msg
  msg/VehicleStatus.msg
)

set(srv_files
)

set(action_files
  "action/SetFlightMode.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
