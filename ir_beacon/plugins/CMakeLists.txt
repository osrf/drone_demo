set (HEADERS
  IRSensorPlugin.hh
)

set (SOURCES
  IRSensorPlugin.cc
)

find_package(gazebo REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_irsensor_plugin SHARED 
  ${HEADERS}
  ${SOURCES}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

target_link_libraries(gazebo_irsensor_plugin ${GAZEBO_libraries} ${catkin_LIBRARIES})

install(TARGETS gazebo_irsensor_plugin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
