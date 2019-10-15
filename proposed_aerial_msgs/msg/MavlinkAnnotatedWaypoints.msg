# targeted for mavros_msgs
# replaces mavros_msgs/WaypointList

# seq nr of currently active waypoint
# waypoints[current_seq].is_current == True
uint16 current_seq

GlobalPosition[] waypoints
# Additional mavlink specific parameters for each waypoint
# This list should be a one-to-one set of parameters for each of the above waypoints.
MavlinkWaypointParameters[] parameters