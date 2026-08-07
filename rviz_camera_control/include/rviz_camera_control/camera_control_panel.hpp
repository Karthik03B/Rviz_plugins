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

#ifndef RVIZ_CAMERA_CONTROL__CAMERA_CONTROL_PANEL_HPP_
#define RVIZ_CAMERA_CONTROL__CAMERA_CONTROL_PANEL_HPP_

#include <memory>
#include <string>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "rviz_camera_control/external_view_controller.hpp"
#include "rviz_camera_control/msg/camera_command.hpp"
#include "rviz_camera_control/msg/camera_state.hpp"

namespace rviz_common
{
class Config;
}  // namespace rviz_common

namespace rviz_camera_control
{

/// @class CameraControlPanel
/// @brief RViz Panel that bridges ROS 2 topics to the RViz render camera.
///
/// Responsibilities:
///  1. Subscribes to a CameraCommand topic. Each message is transformed
///     (via tf2) into RViz's current Fixed Frame and forwarded to the
///     active view controller, but *only* if that controller is an
///     ExternalViewController (class id "rviz_camera_control/External").
///     A button in this panel switches RViz into that view controller.
///  2. Independently of whether external control is currently active,
///     publishes a CameraState message on a timer describing whatever the
///     render camera is actually doing right now -- this is deliberately
///     decoupled from (1) so operator tooling always has ground truth about
///     the RViz viewpoint, even while a human is manually orbiting.
///
/// This class owns the one rclcpp::Node RViz exposes to plugins (obtained
/// through DisplayContext::getRosNodeAbstraction(), the same abstraction
/// every RViz2 ROS-aware plugin uses) and a tf2 buffer/listener used only to
/// transform incoming commands into the Fixed Frame.
class CameraControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CameraControlPanel(QWidget * parent = nullptr);
  ~CameraControlPanel() override;

  /// Wires up the ROS 2 node, tf buffer, subscription, publisher and timer.
  /// Called once by RViz after the DisplayContext becomes available.
  void onInitialize() override;

  /// Persist user-configurable settings (topic names, publish rate) into
  /// the saved .rviz file.
  void save(rviz_common::Config config) const override;

  /// Restore user-configurable settings from a loaded .rviz file.
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  /// Re-creates the subscription/publisher and timer using whatever values
  /// are currently entered in the topic/rate fields.
  void onApplySettings();

  /// Switches RViz's active view controller to rviz_camera_control/External.
  void onActivateExternalView();

  /// Periodic tick: publishes CameraState and refreshes the on-screen
  /// status/pose labels.
  void onTimerTick();

private:
  /// Subscription callback: transforms the commanded pose into the Fixed
  /// Frame with tf2, then forwards it to the active ExternalViewController
  /// (if any is active).
  void cameraCommandCallback(const msg::CameraCommand::ConstSharedPtr msg);

  /// Returns the active view controller cast to ExternalViewController, or
  /// nullptr if a different view controller is currently active.
  ExternalViewController * getActiveExternalController() const;

  /// Builds the Qt widget tree. Called once from the constructor.
  void buildUi();

  /// (Re)creates the ROS 2 subscription and publisher using the topic names
  /// currently stored in command_topic_/state_topic_. Safe to call more
  /// than once; existing subscription/publisher objects are simply replaced
  /// (their destructors clean up the old ones).
  void createRosInterfaces();

  // --- ROS 2 state -----------------------------------------------------
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> rviz_ros_node_;
  rclcpp::Subscription<msg::CameraCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<msg::CameraState>::SharedPtr state_publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Qt widgets --------------------------------------------------------
  QLineEdit * command_topic_edit_;
  QLineEdit * state_topic_edit_;
  QDoubleSpinBox * publish_rate_spin_;
  QPushButton * apply_button_;
  QPushButton * activate_button_;
  QLabel * link_status_label_;
  QLabel * pose_label_;
  QTimer * publish_timer_;

  // --- configuration mirrored to/from Panel::save/load -------------------
  QString command_topic_;
  QString state_topic_;
  double publish_rate_hz_;
};

}  // namespace rviz_camera_control

#endif  // RVIZ_CAMERA_CONTROL__CAMERA_CONTROL_PANEL_HPP_
