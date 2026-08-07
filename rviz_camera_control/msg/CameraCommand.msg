# CameraCommand.msg
#
# Commands the RViz render camera to an absolute pose expressed in
# header.frame_id. The message is only acted on while the RViz view
# controller currently active is "rviz_camera_control/External"
# (see ExternalViewController); otherwise CameraControlPanel logs a
# throttled warning and drops it.
#
# position/orientation follow REP-103 (X forward, Y left, Z up).

std_msgs/Header header

# Desired camera position, in header.frame_id.
geometry_msgs/Point position

# Desired camera orientation, in header.frame_id.
# Must be a valid (non-zero) quaternion; it is re-normalized defensively.
geometry_msgs/Quaternion orientation

# Vertical field of view, in radians. Values <= 0.0 mean "leave unchanged".
float64 fov_y

# Near clip distance, in metres. Values <= 0.0 mean "leave unchanged".
float64 near_clip

# Far clip distance, in metres. Values <= 0.0 mean "leave unchanged".
float64 far_clip
