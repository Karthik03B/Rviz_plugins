# rviz_camera_control

A ROS 2 Humble RViz2 plugin package giving full external (ROS 2 topic driven)
control of the RViz render camera, plus continuous camera-state telemetry.
Two plugin classes, one shared library, no modification to RViz's own source:

- **`rviz_camera_control::ExternalViewController`**
  (`rviz_common::ViewController`, class id `rviz_camera_control/External`) --
  the actual camera-owning object. RViz's view controllers are the one
  extension point that owns the render camera transform; this is why camera
  control is implemented here and not in the panel.
- **`rviz_camera_control::CameraControlPanel`**
  (`rviz_common::Panel`) -- owns the ROS 2 node, subscribes to
  `CameraCommand`, publishes `CameraState`, and gives the operator a button
  to switch RViz into the external view controller.

## Why two plugin classes instead of one

RViz instantiates Panels and ViewControllers independently through separate
pluginlib factories; there is no constructor-time way to wire one to the
other. The panel finds the controller at runtime via
`ViewManager::getCurrent()` + `dynamic_cast`, exactly the same way any other
RViz code inspects the active view controller. This means:

- Commands only take effect while `rviz_camera_control/External` is the
  active view controller (select it from RViz's **Views** panel, or click
  **"Activate External Camera View"** in this panel).
- `CameraState` telemetry is published regardless of which view controller
  is active, so operator tooling always has ground truth about what the
  camera is actually showing, even while a human is manually orbiting with
  the built-in Orbit controller.

## Messages

- `rviz_camera_control/msg/CameraCommand` -- absolute pose command
  (`position`, `orientation`, in `header.frame_id`), plus optional
  `fov_y` / `near_clip` / `far_clip` (values `<= 0.0` mean "leave
  unchanged"). Transformed into RViz's current Fixed Frame via `tf2` before
  being applied, so you can command the camera from any frame that has a
  valid transform to the Fixed Frame (e.g. drive the RViz camera straight
  from your UAV's own pose topic).
- `rviz_camera_control/msg/CameraState` -- what the camera is doing right
  now: `position`, `orientation`, `fov_y`, `near_clip`, `far_clip`,
  `view_controller_class`, `external_control_active`.

Default topics: `/rviz_camera_control/camera_cmd` (subscribed),
`/rviz_camera_control/camera_state` (published). Both, plus the publish
rate, are editable in the panel UI and persisted in the saved `.rviz` file.

## Coordinate conventions -- read this before wiring anything up

`position`/`orientation` in both messages use **ROS/REP-103 convention**
(X forward, Y left, Z up), expressed in RViz's **Fixed Frame**. Internally:

- Position needs no axis remapping: RViz keeps Ogre world-space coordinates
  numerically identical to ROS Fixed Frame coordinates. This is provable
  from `rviz_common::FrameManagerIface::getTransform()`, whose returned
  `Ogre::Vector3`/`Ogre::Quaternion` are used directly with
  `Ogre::SceneNode::setPosition()`/`setOrientation()` throughout RViz's own
  display plugins -- no separate transform is applied in between.
- Orientation needs exactly one fixed *local* rotation, because Ogre's
  camera looks down its own local `-Z` with local `+Y` as "up", while a
  ROS/REP-103 body frame at identity orientation looks down local `+X` with
  local `+Z` as "up". `ExternalViewController` applies the same fixed
  rotation RViz's own built-in `FPSViewController` uses for this exact
  reason (see `kRosToOgreCameraRotation` in
  `external_view_controller.cpp`).

## Build

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws/src
# copy or clone this rviz_camera_control/ directory here
cd ~/uav_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rviz_camera_control
source install/setup.bash
rviz2
```

In RViz: **Panels -> Add New Panel -> rviz_camera_control -> CameraControlPanel**.

## Known engineering caveats -- I did not pretend these away

I do not have a ROS 2 Humble install available to actually run `colcon
build` against real headers in the environment I wrote this in, and this
sandbox's network is locked to package registries (pip/npm/apt security
mirror), not the ROS apt repo, so I could not `apt install ros-humble-rviz*`
and compile-check this. Everything I *could* verify, I pulled from the
actual `ros2/rviz` source and the official ROS 2 Humble tutorial docs
(not from memory) -- see the class-level comments for what's grounded in
source vs. inferred. The one spot with real residual risk:

- **`find_package(rviz_ogre_vendor REQUIRED)`** in `CMakeLists.txt`. I'm
  confident this vendor package is how Humble's `rviz_rendering` gets Ogre
  headers, but I could not directly diff the exact CMake target names
  against your installed Humble packages. If `colcon build` fails on the
  Ogre `find_package` step specifically (not on anything in this package's
  own C++), open `/opt/ros/humble/share/rviz_rendering/cmake/` or that
  package's `CMakeLists.txt` on your machine and match its `find_package`
  calls -- everything else in this file (the plugin library itself, the
  message generation, the pluginlib export) does not depend on that being
  exactly right.
- `ExternalViewController::lookAt()` is implemented and correct by
  construction, but I have not been able to click-test RViz's double-click
  focus interaction against it. If it feels off, the pose math is isolated
  in `computeLookAtOrientationRos()` -- that's the only place to look.
- I did not implement mouse-drag interaction for `ExternalViewController`
  (no `handleMouseEvent` override) because the spec is "complete *external*
  control" -- mixing manual mouse control into the same controller that a
  ROS topic is also driving would fight itself. If you later want a hybrid
  mode (mouse nudges while idle, ROS command when a message arrives), that's
  a real feature to design, not a one-line addition -- say so and I'll scope
  it properly instead of bolting it on.

## Extending toward the UAV operator interface

The natural next steps, each a separate scoped piece of work rather than
something to guess at now:
- A small ROS 2 node (or MAVROS bridge) that republishes vehicle pose (or a
  chase-camera offset from it) as `CameraCommand` at your control loop rate.
- A `follow_mode` field or a second topic for "attach to TF frame with
  offset" instead of always requiring absolute pose -- straightforward to
  add to `ExternalViewController` once you know whether you want that logic
  in this package or upstream in whatever publishes the pose.
- FOV/clip animation (smooth zoom) -- currently `setCommandedFov()` is a
  hard cut; if you want eased transitions that's a small state machine to
  add to `onTimerTick`'s counterpart on the controller side, not here.
