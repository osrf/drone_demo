cmake_minimum_required(VERSION 3.5)
project(px4)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

set(PX4_TARGET px4_sitl_rtps)

set(firmware_path "${CMAKE_CURRENT_SOURCE_DIR}/Firmware")
ExternalProject_Add(firmware
  GIT_REPOSITORY https://github.com/PX4/Firmware.git
  GIT_TAG master
  # UPDATE_COMMAND "${firmware_path}/Tools/check_submodules.sh"
  # SOURCE_DIR "${firmware_path}/Tools/sitl_gazebo"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
  INSTALL_COMMAND rsync -avL --exclude .git --exclude build . build/${PX4_TARGET}/bin ${CMAKE_INSTALL_PREFIX}/share/px4
  CONFIGURE_COMMAND wget -q https://raw.githubusercontent.com/PX4/Firmware/master/msg/tools/uorb_rtps_message_ids.yaml -O uorb_rtps_message_ids.yaml
                    && patch --batch uorb_rtps_message_ids.yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/uorb_rtps_message_ids.patch -o ./msg/tools/uorb_rtps_message_ids.yaml
  BUILD_COMMAND make ${PX4_TARGET}
  BUILD_IN_SOURCE ON
)

ament_environment_hooks(cmake/99_px4_setup.sh.in)

ament_export_libraries(px4)

ament_package()
