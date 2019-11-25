# This represents an estimate of a position, velocity, and acceleration in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The velocity in this message should be specified in the coordinate frame given by the child_frame_id.
# The acceleration in this message should be specified in the coordinate frame given by the child_frame_id.
std_msgs/Header header
string child_frame_id
geometry_msgs/Pose pose
UpperTriangularCovariance pose_covariance
geometry_msgs/Twist velocity
UpperTriangularCovariance velocity_covariance
geometry_msgs/Twist acceleration
UpperTriangularCovariance acceleration_covariance
