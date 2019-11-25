cmake_minimum_required(VERSION 3.5)
project(qgroundcontrol)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

ExternalProject_Add(qgroundcontrol
  # GIT_REPOSITORY https://github.com/mavlink/qgroundcontrol.git
  # GIT_TAG master
  URL https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.tar.bz2
  # UPDATE_COMMAND "${firmware_path}/Tools/check_submodules.sh"
  # SOURCE_DIR "${firmware_path}/Tools/sitl_gazebo"
  # CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND echo ${CMAKE_INSTALL_PREFIX} && mkdir -p ${CMAKE_INSTALL_PREFIX}/lib && cp -r ../qgroundcontrol ${CMAKE_INSTALL_PREFIX}/lib
)

ament_export_libraries(qgroundcontrol)

ament_package()
