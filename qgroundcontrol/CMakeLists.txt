cmake_minimum_required(VERSION 2.8.3)
project(qgroundcontrol)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qgroundcontrol
#  CATKIN_DEPENDS tar unzip
#  DEPENDS system_lib
)

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
  INSTALL_COMMAND echo ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION} && cp -r ../qgroundcontrol ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
)

