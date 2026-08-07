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

#include "rviz_camera_control/camera_control_panel.hpp"

#include <algorithm>
#include <functional>

#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <OgreCamera.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>

namespace rviz_camera_control
{

namespace
{
constexpr char kExternalViewControllerClassId[] = "rviz_camera_control/External";
constexpr char kDefaultCommandTopic[] = "/rviz_camera_control/camera_cmd";
constexpr char kDefaultStateTopic[] = "/rviz_camera_control/camera_state";
constexpr double kDefaultPublishRateHz = 30.0;
constexpr double kMinPublishRateHz = 1.0;
constexpr double kMaxPublishRateHz = 200.0;
}  // namespace

CameraControlPanel::CameraControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  command_topic_edit_(nullptr),
  state_topic_edit_(nullptr),
  publish_rate_spin_(nullptr),
  apply_button_(nullptr),
  activate_button_(nullptr),
  link_status_label_(nullptr),
  pose_label_(nullptr),
  publish_timer_(nullptr),
  command_topic_(kDefaultCommandTopic),
  state_topic_(kDefaultStateTopic),
  publish_rate_hz_(kDefaultPublishRateHz)
{
  buildUi();
}

CameraControlPanel::~CameraControlPanel() = default;

void CameraControlPanel::buildUi()
{
  auto * main_layout = new QVBoxLayout(this);

  // --- Topic / rate configuration group ---
  auto * topics_box = new QGroupBox("ROS 2 Topics", this);
  auto * form_layout = new QFormLayout(topics_box);

  command_topic_edit_ = new QLineEdit(command_topic_, topics_box);
  state_topic_edit_ = new QLineEdit(state_topic_, topics_box);

  publish_rate_spin_ = new QDoubleSpinBox(topics_box);
  publish_rate_spin_->setRange(kMinPublishRateHz, kMaxPublishRateHz);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setSuffix(" Hz");
  publish_rate_spin_->setValue(publish_rate_hz_);

  form_layout->addRow("Command topic", command_topic_edit_);
  form_layout->addRow("State topic", state_topic_edit_);
  form_layout->addRow("Publish rate", publish_rate_spin_);

  apply_button_ = new QPushButton("Apply", topics_box);
  form_layout->addRow(apply_button_);

  main_layout->addWidget(topics_box);

  // --- Camera control / status group ---
  auto * control_box = new QGroupBox("Camera Control", this);
  auto * control_layout = new QVBoxLayout(control_box);

  activate_button_ = new QPushButton("Activate External Camera View", control_box);
  link_status_label_ = new QLabel("View controller: unknown", control_box);
  pose_label_ = new QLabel("Position: --\nOrientation: --", control_box);
  pose_label_->setWordWrap(true);

  control_layout->addWidget(activate_button_);
  control_layout->addWidget(link_status_label_);
  control_layout->addWidget(pose_label_);

  main_layout->addWidget(control_box);
  main_layout->addStretch(1);

  setLayout(main_layout);

  connect(apply_button_, &QPushButton::clicked, this, &CameraControlPanel::onApplySettings);
  connect(
    activate_button_, &QPushButton::clicked, this, &CameraControlPanel::onActivateExternalView);
}

void CameraControlPanel::onInitialize()
{
  // getRosNodeAbstraction() returns a weak_ptr; lock() gives us shared
  // ownership of the one rclcpp::Node RViz manages internally. This is the
  // documented ROS 2 access pattern for RViz panels.
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction().lock();

  createRosInterfaces();

  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &CameraControlPanel::onTimerTick);
  publish_timer_->start(static_cast<int>(1000.0 / publish_rate_hz_));
}

void CameraControlPanel::createRosInterfaces()
{
  if (!rviz_ros_node_) {
    return;
  }
  rclcpp::Node::SharedPtr node = rviz_ros_node_->get_raw_node();

  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Replacing these shared_ptrs releases the previous subscription/publisher
  // (their destructors unregister with the middleware) before the new ones
  // are created under (possibly) different topic names.
  command_subscription_ = node->create_subscription<msg::CameraCommand>(
    command_topic_.toStdString(), rclcpp::QoS(10),
    std::bind(&CameraControlPanel::cameraCommandCallback, this, std::placeholders::_1));

  state_publisher_ = node->create_publisher<msg::CameraState>(
    state_topic_.toStdString(), rclcpp::QoS(10));
}

void CameraControlPanel::onApplySettings()
{
  command_topic_ = command_topic_edit_->text();
  state_topic_ = state_topic_edit_->text();
  publish_rate_hz_ = std::max(kMinPublishRateHz, publish_rate_spin_->value());

  createRosInterfaces();

  if (publish_timer_ != nullptr) {
    publish_timer_->start(static_cast<int>(1000.0 / publish_rate_hz_));
  }

  // Tell RViz this panel's saved configuration has changed, so a subsequent
  // "Save Config" picks up the new topic names / rate.
  Q_EMIT configChanged();
}

void CameraControlPanel::onActivateExternalView()
{
  if (getDisplayContext() == nullptr) {
    return;
  }
  getDisplayContext()->getViewManager()->setCurrentViewControllerType(
    QString(kExternalViewControllerClassId));
}

ExternalViewController * CameraControlPanel::getActiveExternalController() const
{
  if (getDisplayContext() == nullptr) {
    return nullptr;
  }
  rviz_common::ViewController * current = getDisplayContext()->getViewManager()->getCurrent();
  return dynamic_cast<ExternalViewController *>(current);
}

void CameraControlPanel::cameraCommandCallback(const msg::CameraCommand::ConstSharedPtr msg)
{
  ExternalViewController * controller = getActiveExternalController();
  if (controller == nullptr) {
    RCLCPP_WARN_THROTTLE(
      rviz_ros_node_->get_raw_node()->get_logger(),
      *rviz_ros_node_->get_raw_node()->get_clock(),
      5000,
      "Received CameraCommand but the active RViz view controller is not "
      "'%s'. Click 'Activate External Camera View' in this panel, or select "
      "it from RViz's Views panel, before sending commands.",
      kExternalViewControllerClassId);
    return;
  }

  const std::string fixed_frame = getDisplayContext()->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = msg->header;
  pose_in.pose.position = msg->position;
  pose_in.pose.orientation = msg->orientation;

  geometry_msgs::msg::PoseStamped pose_out;
  try {
    if (msg->header.frame_id.empty() || msg->header.frame_id == fixed_frame) {
      // Already in the Fixed Frame (or no frame given, which we treat the
      // same way) -- skip the tf2 lookup entirely.
      pose_out = pose_in;
      pose_out.header.frame_id = fixed_frame;
    } else {
      pose_out = tf_buffer_->transform(pose_in, fixed_frame, tf2::durationFromSec(0.1));
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      rviz_ros_node_->get_raw_node()->get_logger(),
      "CameraCommand: failed to transform pose from '%s' to Fixed Frame '%s': %s",
      msg->header.frame_id.c_str(), fixed_frame.c_str(), ex.what());
    return;
  }

  const Ogre::Vector3 position_ros(
    static_cast<Ogre::Real>(pose_out.pose.position.x),
    static_cast<Ogre::Real>(pose_out.pose.position.y),
    static_cast<Ogre::Real>(pose_out.pose.position.z));

  const Ogre::Quaternion orientation_ros(
    static_cast<Ogre::Real>(pose_out.pose.orientation.w),
    static_cast<Ogre::Real>(pose_out.pose.orientation.x),
    static_cast<Ogre::Real>(pose_out.pose.orientation.y),
    static_cast<Ogre::Real>(pose_out.pose.orientation.z));

  controller->setCommandedPose(position_ros, orientation_ros);
  controller->setCommandedFov(msg->fov_y);
  controller->setCommandedClipDistances(msg->near_clip, msg->far_clip);
}

void CameraControlPanel::onTimerTick()
{
  if (getDisplayContext() == nullptr || !state_publisher_) {
    return;
  }

  rviz_common::ViewController * current = getDisplayContext()->getViewManager()->getCurrent();
  if (current == nullptr || current->getCamera() == nullptr) {
    link_status_label_->setText("View controller: none");
    return;
  }

  ExternalViewController * external_controller = dynamic_cast<ExternalViewController *>(current);

  Ogre::Vector3 position_ros;
  Ogre::Quaternion orientation_ros;

  if (external_controller != nullptr) {
    // Preferred path: ask the controller for the ROS-convention pose
    // directly, so the axis-remap logic lives in exactly one place.
    external_controller->getCurrentPoseRos(position_ros, orientation_ros);
    link_status_label_->setText(
      QString("View controller: %1  (external control ACTIVE)").arg(current->getClassId()));
  } else {
    // Any other (built-in) view controller: we still want to publish
    // telemetry, but we do not attempt to convert its camera orientation
    // back to ROS convention, because that fixed local-frame conversion is
    // only proven correct for cameras this package itself drives. Position
    // is still meaningful because Ogre world-space coordinates equal Fixed
    // Frame coordinates regardless of which controller is active.
    Ogre::Camera * camera = current->getCamera();
    position_ros = camera->getPosition();
    orientation_ros = camera->getOrientation();
    link_status_label_->setText(
      QString("View controller: %1  (external control inactive)").arg(current->getClassId()));
  }

  pose_label_->setText(
    QString("Position: [%1, %2, %3]\nOrientation (w,x,y,z): [%4, %5, %6, %7]")
    .arg(position_ros.x, 0, 'f', 3).arg(position_ros.y, 0, 'f', 3)
    .arg(position_ros.z, 0, 'f', 3)
    .arg(orientation_ros.w, 0, 'f', 3).arg(orientation_ros.x, 0, 'f', 3)
    .arg(orientation_ros.y, 0, 'f', 3).arg(orientation_ros.z, 0, 'f', 3));

  Ogre::Camera * camera = current->getCamera();

  msg::CameraState state_msg;
  state_msg.header.stamp = rviz_ros_node_->get_raw_node()->get_clock()->now();
  state_msg.header.frame_id = getDisplayContext()->getFixedFrame().toStdString();
  state_msg.reference_frame = state_msg.header.frame_id;
  state_msg.position.x = position_ros.x;
  state_msg.position.y = position_ros.y;
  state_msg.position.z = position_ros.z;
  state_msg.orientation.w = orientation_ros.w;
  state_msg.orientation.x = orientation_ros.x;
  state_msg.orientation.y = orientation_ros.y;
  state_msg.orientation.z = orientation_ros.z;
  state_msg.fov_y = camera->getFOVy().valueRadians();
  state_msg.near_clip = camera->getNearClipDistance();
  state_msg.far_clip = camera->getFarClipDistance();
  state_msg.view_controller_class = current->getClassId().toStdString();
  state_msg.external_control_active = (external_controller != nullptr);

  state_publisher_->publish(state_msg);
}

void CameraControlPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("CommandTopic", command_topic_);
  config.mapSetValue("StateTopic", state_topic_);
  config.mapSetValue("PublishRateHz", publish_rate_hz_);
}

void CameraControlPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);

  QString topic;
  bool needs_rebind = false;

  if (config.mapGetString("CommandTopic", &topic)) {
    command_topic_ = topic;
    needs_rebind = true;
  }
  if (config.mapGetString("StateTopic", &topic)) {
    state_topic_ = topic;
    needs_rebind = true;
  }

  float rate = 0.0f;
  if (config.mapGetFloat("PublishRateHz", &rate) && rate > 0.0f) {
    publish_rate_hz_ = rate;
  }

  // Reflect restored values in the widgets (widgets already exist because
  // buildUi() runs from the constructor, which always runs before load()).
  if (command_topic_edit_ != nullptr) {
    command_topic_edit_->setText(command_topic_);
  }
  if (state_topic_edit_ != nullptr) {
    state_topic_edit_->setText(state_topic_);
  }
  if (publish_rate_spin_ != nullptr) {
    publish_rate_spin_->setValue(publish_rate_hz_);
  }

  // load() can run either before or after onInitialize() depending on
  // whether this panel is being restored from a saved .rviz file. If the
  // ROS node is already available, rebind immediately so the restored topic
  // names actually take effect instead of silently keeping the defaults
  // onInitialize() bound at construction time.
  if (needs_rebind && rviz_ros_node_) {
    createRosInterfaces();
  }
}

}  // namespace rviz_camera_control

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_camera_control::CameraControlPanel, rviz_common::Panel)
