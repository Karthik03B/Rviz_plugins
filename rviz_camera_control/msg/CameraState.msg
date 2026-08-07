# CameraState.msg
#
# Continuous telemetry describing exactly what the RViz render camera is
# doing right now, published by CameraControlPanel regardless of which view
# controller is active (Orbit, FPS, External, ...). Use
# external_control_active to tell whether CameraCommand messages are
# currently being honored.
#
# position/orientation follow REP-103 (X forward, Y left, Z up) and are
# expressed in reference_frame (RViz's current Fixed Frame).

std_msgs/Header header

# Name of the RViz Fixed Frame that position/orientation are expressed in.
string reference_frame

geometry_msgs/Point position
geometry_msgs/Quaternion orientation

float64 fov_y
float64 near_clip
float64 far_clip

# Class id of the view controller currently active in RViz, e.g.
# "rviz_default_plugins/Orbit" or "rviz_camera_control/External".
string view_controller_class

# True only when view_controller_class is the External controller, i.e.
# CameraCommand messages are actively being applied to the camera.
bool external_control_active
