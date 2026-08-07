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

#include "rviz_camera_control/external_view_controller.hpp"

#include <cmath>

#include <OgreCamera.h>
#include <OgreMatrix3.h>

namespace rviz_camera_control
{

// See the header-file comment on kRosToOgreCameraRotation for the rationale.
// This is composed identically to rviz's own FPSViewController:
//   Ry(-90deg) * Rz(-90deg)
const Ogre::Quaternion ExternalViewController::kRosToOgreCameraRotation =
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

ExternalViewController::ExternalViewController()
: position_property_(nullptr),
  orientation_property_(nullptr),
  status_property_(nullptr)
{
  // These are added as child Property objects of this ViewController, which
  // is itself a Property; RViz's Views panel renders them as an expandable
  // tree automatically. This mirrors the pattern used by every built-in
  // view controller (Orbit's "Distance"/"Focal Point" properties, etc.).
  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Current camera position in the Fixed Frame (ROS convention). "
    "Read-only telemetry -- edit via CameraCommand messages, not here.",
    this);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY,
    "Current camera orientation in the Fixed Frame (ROS convention). "
    "Read-only telemetry -- edit via CameraCommand messages, not here.",
    this);

  status_property_ = new rviz_common::properties::StringProperty(
    "Status", "Waiting for external command",
    "State of the external command link.",
    this);
}

ExternalViewController::~ExternalViewController() = default;

void ExternalViewController::onInitialize()
{
  // near_clip_property_ already exists on the base ViewController class and
  // is wired to camera_->setNearClipDistance() by the base implementation;
  // we still set an explicit default here so the camera is valid before any
  // property change fires.
  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  camera_->setFOVy(Ogre::Radian(Ogre::Degree(50.0f)));
  camera_->setNearClipDistance(0.01f);
  camera_->setFarClipDistance(1000.0f);
  reset();
}

void ExternalViewController::reset()
{
  const Ogre::Vector3 default_position_ros(-5.0f, 0.0f, 2.0f);
  const Ogre::Quaternion default_orientation_ros =
    computeLookAtOrientationRos(default_position_ros, Ogre::Vector3::ZERO);
  setCommandedPose(default_position_ros, default_orientation_ros);
  status_property_->setStdString("Reset to default pose");
}

void ExternalViewController::lookAt(const Ogre::Vector3 & point)
{
  if (camera_ == nullptr) {
    return;
  }
  // camera_->getPosition() lives in the same numeric world space that
  // setCommandedPose() writes to (see the header comment on coordinate
  // conventions), so it can be fed straight back in without conversion.
  const Ogre::Vector3 eye_ros = camera_->getPosition();
  const Ogre::Quaternion orientation_ros = computeLookAtOrientationRos(eye_ros, point);
  setCommandedPose(eye_ros, orientation_ros);
}

void ExternalViewController::setCommandedPose(
  const Ogre::Vector3 & position_ros, const Ogre::Quaternion & orientation_ros)
{
  if (camera_ == nullptr) {
    return;
  }

  Ogre::Quaternion normalized_orientation = orientation_ros;
  normalized_orientation.normalise();

  camera_->setPosition(position_ros);
  camera_->setOrientation(normalized_orientation * kRosToOgreCameraRotation);

  position_property_->setVector(position_ros);
  orientation_property_->setQuaternion(normalized_orientation);
  status_property_->setStdString("Tracking external command");
}

void ExternalViewController::setCommandedFov(double fov_y_radians)
{
  if (camera_ == nullptr || fov_y_radians <= 0.0) {
    return;
  }
  camera_->setFOVy(Ogre::Radian(static_cast<Ogre::Real>(fov_y_radians)));
}

void ExternalViewController::setCommandedClipDistances(double near_clip, double far_clip)
{
  if (camera_ == nullptr) {
    return;
  }
  if (near_clip > 0.0) {
    camera_->setNearClipDistance(static_cast<Ogre::Real>(near_clip));
  }
  if (far_clip > 0.0) {
    camera_->setFarClipDistance(static_cast<Ogre::Real>(far_clip));
  }
}

void ExternalViewController::getCurrentPoseRos(
  Ogre::Vector3 & position_ros, Ogre::Quaternion & orientation_ros) const
{
  if (camera_ == nullptr) {
    position_ros = Ogre::Vector3::ZERO;
    orientation_ros = Ogre::Quaternion::IDENTITY;
    return;
  }
  position_ros = camera_->getPosition();
  orientation_ros = camera_->getOrientation() * kRosToOgreCameraRotation.Inverse();
  orientation_ros.normalise();
}

Ogre::Quaternion ExternalViewController::computeLookAtOrientationRos(
  const Ogre::Vector3 & eye_ros, const Ogre::Vector3 & target_ros)
{
  Ogre::Vector3 forward = target_ros - eye_ros;
  if (forward.squaredLength() < 1e-12f) {
    // Degenerate: target coincides with eye. Keep whatever orientation the
    // caller already had rather than producing a NaN quaternion.
    return Ogre::Quaternion::IDENTITY;
  }
  forward.normalise();

  const Ogre::Vector3 world_up = Ogre::Vector3::UNIT_Z;

  // Degenerate case: looking almost straight up or down along world Z --
  // cross(world_up, forward) would be near-zero, so pick a different
  // reference axis to derive "left"/"up" from.
  Ogre::Vector3 reference_up = world_up;
  if (std::abs(forward.dotProduct(world_up)) > 0.999f) {
    reference_up = Ogre::Vector3::UNIT_X;
  }

  // Right-handed ROS/REP-103 body frame: X forward, Y left, Z up, i.e.
  // X (forward) cross Y (left) = Z (up). Solving for a left/up pair that is
  // orthogonal to "forward" and as close as possible to reference_up:
  Ogre::Vector3 left = reference_up.crossProduct(forward);
  left.normalise();
  Ogre::Vector3 up = forward.crossProduct(left);
  up.normalise();

  Ogre::Matrix3 basis;
  basis.SetColumn(0, forward);
  basis.SetColumn(1, left);
  basis.SetColumn(2, up);

  Ogre::Quaternion orientation_ros;
  orientation_ros.FromRotationMatrix(basis);
  orientation_ros.normalise();
  return orientation_ros;
}

}  // namespace rviz_camera_control

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_camera_control::ExternalViewController, rviz_common::ViewController)
