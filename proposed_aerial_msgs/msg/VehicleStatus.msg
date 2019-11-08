# This message represents vehicle information

# HIL
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

uint8 VEHICLE_TYPE_UNKNOWN = 0
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3
uint8 VEHICLE_TYPE_VTOL = 4

std_msgs/Header header

uint8 system_type     # system type
bool engine_failure   # Set to true if an engine failure is detected
bool failsafe         # true if system is in failsafe state
uint8 hil_state       # current hil state
