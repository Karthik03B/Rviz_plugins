// Copyright (c) 2026, Karthik.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RVIZ_CAMERA_CONTROL__EXTERNAL_VIEW_CONTROLLER_HPP_
#define RVIZ_CAMERA_CONTROL__EXTERNAL_VIEW_CONTROLLER_HPP_

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/properties/quaternion_property.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace rviz_camera_control
{

/// @class ExternalViewController
/// @brief An rviz_common::ViewController whose pose is set entirely by an
///        external caller instead of by mouse interaction.
///
/// Select this view controller from RViz's "Views" panel (class id
/// "rviz_camera_control/External"), or have CameraControlPanel switch to it
/// programmatically, to hand full control of the render camera to a ROS 2
/// topic. This is the correct RViz extension point for this job: view
/// controllers, not displays or tools, own the render camera transform, and
/// RViz recomputes nothing about the camera unless this class tells it to.
///
/// Coordinate conventions
/// -----------------------
/// All position/orientation values accepted or returned by this class use
/// ROS / REP-103 convention (X forward, Y left, Z up), expressed in RViz's
/// current Fixed Frame. Internally:
///  - Position requires no axis remapping. RViz keeps world-space Ogre
///    coordinates numerically identical to ROS Fixed Frame coordinates --
///    this is verifiable from rviz_common::FrameManagerIface::getTransform(),
///    whose returned Ogre::Vector3/Ogre::Quaternion are used directly with
///    Ogre::SceneNode::setPosition()/setOrientation() throughout RViz's own
///    display plugins, with no separate transform applied in between.
///  - Orientation requires one fixed *local* rotation, because Ogre's camera
///    looks down its own local -Z axis with local +Y as "up", whereas a
///    ROS/REP-103 body frame's identity orientation looks down local +X
///    with local +Z as "up". This class applies the same fixed conversion
///    RViz's own FPSViewController uses for the same reason (see
///    kRosToOgreCameraRotation below).
class ExternalViewController : public rviz_common::ViewController
{
  Q_OBJECT

public:
  ExternalViewController();
  ~ExternalViewController() override;

  /// Command the camera to a new position and orientation.
  /// @param position_ros     Position in the Fixed Frame, ROS convention.
  /// @param orientation_ros  Orientation in the Fixed Frame, ROS convention.
  ///                         Does not need to already be normalized.
  void setCommandedPose(
    const Ogre::Vector3 & position_ros,
    const Ogre::Quaternion & orientation_ros);

  /// Command a new vertical field of view. Values <= 0 are ignored, so
  /// callers can pass "0" from an optional message field to mean
  /// "leave unchanged".
  void setCommandedFov(double fov_y_radians);

  /// Command new near/far clip distances. Values <= 0 are ignored per-field.
  void setCommandedClipDistances(double near_clip, double far_clip);

  /// Read back the camera's current position/orientation, converted back
  /// into ROS/REP-103 convention. Used by CameraControlPanel to publish
  /// CameraState telemetry without duplicating the axis-remap logic here.
  void getCurrentPoseRos(
    Ogre::Vector3 & position_ros,
    Ogre::Quaternion & orientation_ros) const;

protected:
  /// Called once by the ViewController base class after context_ and
  /// camera_ have been constructed. Sets sane default projection parameters
  /// and puts the camera at a visible default pose.
  void onInitialize() override;

  /// Put the camera into a fixed, predictable default pose: 5 m back along
  /// -X and 2 m up, looking at the Fixed Frame origin.
  void reset() override;

  /// Re-aim the camera at a Fixed-Frame point while keeping its current
  /// position. RViz calls this on some built-in interactions (e.g.
  /// double-click focus); it is not used by the ROS command path, but is
  /// implemented so this controller behaves predictably if a user interacts
  /// with it directly.
  void lookAt(const Ogre::Vector3 & point) override;

private:
  /// Fixed local-frame rotation converting a ROS/REP-103 orientation
  /// (X forward, Z up) into the orientation Ogre::Camera expects internally
  /// (looks down local -Z, local +Y is up). Numerically identical to the
  /// ROBOT_TO_CAMERA_ROTATION constant used by rviz's own
  /// FPSViewController for the same purpose:
  ///   Rz(-90 deg) about Ogre's local Z, then Ry(-90 deg) about local Y,
  ///   composed as Ry * Rz.
  static const Ogre::Quaternion kRosToOgreCameraRotation;

  /// Build a ROS/REP-103-convention orientation quaternion for a camera
  /// located at eye_ros and facing target_ros, keeping "up" as close to
  /// world +Z as the look direction allows (falls back to world +X as the
  /// reference axis when looking nearly straight up or down).
  static Ogre::Quaternion computeLookAtOrientationRos(
    const Ogre::Vector3 & eye_ros,
    const Ogre::Vector3 & target_ros);

  /// Read-only telemetry properties shown in the Views panel while this
  /// controller is active. They are refreshed on every accepted command,
  /// but editing them by hand in the GUI has no effect on the camera --
  /// this class is the source of truth, not these properties.
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::StringProperty * status_property_;
};

}  // namespace rviz_camera_control

#endif  // RVIZ_CAMERA_CONTROL__EXTERNAL_VIEW_CONTROLLER_HPP_
