//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "include/autoware_state_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <qcolor.h>
#include <qscrollarea.h>
#include <QFile>
#include <QTextStream>

#include <memory>
#include <string>

namespace rviz_plugins
{
AutowareStatePanel::AutowareStatePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Panel Configuration
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // Layout

  // Create a new container widget
  QWidget * containerWidget = new QWidget(this);
  containerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  containerWidget->setStyleSheet(
    QString("QWidget { background-color: %1; color: %2; }")
      .arg(autoware::state_rviz_plugin::colors::default_colors.background.c_str())
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_surface.c_str()));

  auto * containerLayout = new QVBoxLayout(containerWidget);
  // Set the alignment of the layout
  containerLayout->setAlignment(Qt::AlignTop);
  containerLayout->setSpacing(1);

  auto * operation_mode_group = makeOperationModeGroup();
  auto * diagnostic_v_layout = new QVBoxLayout;
  auto * localization_group = makeLocalizationGroup();
  auto * motion_group = makeMotionGroup();
  auto * fail_safe_group = makeFailSafeGroup();
  auto * routing_group = makeRoutingGroup();
  auto * velocity_limit_group = makeVelocityLimitGroup();
  // auto * diagnostic_group = makeDiagnosticGroup();

  diagnostic_v_layout->addLayout(routing_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(localization_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(motion_group);
  // diagnostic_v_layout->addSpacing(5);
  diagnostic_v_layout->addLayout(fail_safe_group);

  // containerLayout->addLayout(diagnostic_group);

  containerLayout->addLayout(operation_mode_group);
  // containerLayout->addSpacing(5);
  containerLayout->addLayout(diagnostic_v_layout);
  // main_v_layout->addSpacing(5);
  containerLayout->addLayout(velocity_limit_group);

  // Create a QScrollArea
  QScrollArea * scrollArea = new QScrollArea(this);
  scrollArea->setWidgetResizable(true);
  scrollArea->setWidget(containerWidget);

  // Main layout for AutowareStatePanel
  QVBoxLayout * mainLayout = new QVBoxLayout(this);
  mainLayout->addWidget(scrollArea);
  setLayout(mainLayout);
}

void AutowareStatePanel::onInitialize()
{
  using std::placeholders::_1;

  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Operation Mode
  sub_operation_mode_ = raw_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onOperationMode, this, _1));

  client_change_to_autonomous_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  client_change_to_stop_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");

  client_change_to_local_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_local");

  client_change_to_remote_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_remote");

  client_enable_autoware_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/enable_autoware_control");

  client_enable_direct_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/disable_autoware_control");

  // Routing
  sub_route_ = raw_node_->create_subscription<RouteState>(
    "/api/routing/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onRoute, this, _1));

  client_clear_route_ = raw_node_->create_client<ClearRoute>("/api/routing/clear_route");

  // Localization
  sub_localization_ = raw_node_->create_subscription<LocalizationInitializationState>(
    "/api/localization/initialization_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onLocalization, this, _1));

  client_init_by_gnss_ =
    raw_node_->create_client<InitializeLocalization>("/api/localization/initialize");

  // Motion
  sub_motion_ = raw_node_->create_subscription<MotionState>(
    "/api/motion/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onMotion, this, _1));

  client_accept_start_ = raw_node_->create_client<AcceptStart>("/api/motion/accept_start");

  // FailSafe
  sub_mrm_ = raw_node_->create_subscription<MRMState>(
    "/api/fail_safe/mrm_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStatePanel::onMRMState, this, _1));

  // // Diagnostics
  // sub_diag_ = raw_node_->create_subscription<DiagnosticArray>(
  //   "/diagnostics", 10, std::bind(&AutowareStatePanel::onDiagnostics, this, _1));

  sub_emergency_ = raw_node_->create_subscription<tier4_external_api_msgs::msg::Emergency>(
    "/api/autoware/get/emergency", 10, std::bind(&AutowareStatePanel::onEmergencyStatus, this, _1));

  client_emergency_stop_ = raw_node_->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "/api/autoware/set/emergency");

  pub_velocity_limit_ = raw_node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity_default", rclcpp::QoS{1}.transient_local());

  // Multiple Goal Pose setup
  sub_goal_pose_ = raw_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS{1},
    std::bind(&AutowareStatePanel::onGoalPose, this, _1));

  pub_goal_pose_ = raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", rclcpp::QoS{1}.transient_local());

  QObject::connect(segmented_button, &CustomSegmentedButton::buttonClicked, this, [this](int id) {
    const QList<QAbstractButton *> buttons = segmented_button->getButtonGroup()->buttons();

    // Check if the button ID is within valid range
    if (id < 0 || id >= buttons.size()) {
      return;
    }

    // Ensure the button is not null
    QAbstractButton * abstractButton = segmented_button->getButtonGroup()->button(id);
    if (!abstractButton) {
      return;
    }

    const QPushButton * button = qobject_cast<QPushButton *>(abstractButton);
    if (button) {
      // Call the corresponding function for each button
      if (button == auto_button_ptr_) {
        onClickAutonomous();
      } else if (button == local_button_ptr_) {
        onClickLocal();
      } else if (button == remote_button_ptr_) {
        onClickRemote();
      } else if (button == stop_button_ptr_) {
        onClickStop();
      }
    } else {
      // qDebug() << "Button not found with ID:" << id;
    }
  });
}

QVBoxLayout * AutowareStatePanel::makeOperationModeGroup()
{
  control_mode_switch_ptr_ = new CustomToggleSwitch(this);
  connect(
    control_mode_switch_ptr_, &QCheckBox::stateChanged, this,
    &AutowareStatePanel::onSwitchStateChanged);

  control_mode_label_ptr_ = new QLabel("Autoware Control");
  control_mode_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));

  CustomContainer * group1 = new CustomContainer(this);

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(control_mode_switch_ptr_);
  horizontal_layout->addWidget(control_mode_label_ptr_);

  // add switch and label to the container
  group1->setContentsMargins(0, 0, 0, 10);
  group1->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);

  // Create the CustomSegmentedButton
  segmented_button = new CustomSegmentedButton(this);
  auto_button_ptr_ = segmented_button->addButton("Auto");
  local_button_ptr_ = segmented_button->addButton("Local");
  remote_button_ptr_ = segmented_button->addButton("Remote");
  stop_button_ptr_ = segmented_button->addButton("Stop");

  QVBoxLayout * groupLayout = new QVBoxLayout;
  // set these widgets to show up at the left and not stretch more than needed
  groupLayout->setAlignment(Qt::AlignCenter);
  groupLayout->setContentsMargins(10, 0, 0, 0);
  groupLayout->addWidget(group1);
  // groupLayout->addSpacing(5);
  groupLayout->addWidget(segmented_button, 0, Qt::AlignCenter);
  return groupLayout;
}

QVBoxLayout * AutowareStatePanel::makeRoutingGroup()
{
  auto * group = new QVBoxLayout;

  auto * custom_container = new CustomContainer(this);

  routing_icon = new CustomIconLabel(
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()));

  clear_route_button_ptr_ = new CustomElevatedButton("Clear Route");
  clear_route_button_ptr_->setCheckable(true);
  clear_route_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(clear_route_button_ptr_, SIGNAL(clicked()), SLOT(onClickClearRoute()));

  routing_label_ptr_ = new QLabel("Routing | Unknown");
  routing_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(routing_icon);
  horizontal_layout->addWidget(routing_label_ptr_);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(clear_route_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);

  return group;
}

QVBoxLayout * AutowareStatePanel::makeLocalizationGroup()
{
  auto * group = new QVBoxLayout;
  auto * custom_container = new CustomContainer(this);

  init_by_gnss_button_ptr_ = new CustomElevatedButton("Initialize with GNSS");
  init_by_gnss_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(init_by_gnss_button_ptr_, SIGNAL(clicked()), SLOT(onClickInitByGnss()));

  localization_icon = new CustomIconLabel(
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()));
  localization_label_ptr_ = new QLabel("Localization | Unknown");
  localization_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(localization_icon);
  horizontal_layout->addWidget(localization_label_ptr_);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(init_by_gnss_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);
  return group;
}

QVBoxLayout * AutowareStatePanel::makeMotionGroup()
{
  auto * group = new QVBoxLayout;
  auto * custom_container = new CustomContainer(this);

  accept_start_button_ptr_ = new CustomElevatedButton("Accept Start");
  accept_start_button_ptr_->setCheckable(true);
  accept_start_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(accept_start_button_ptr_, SIGNAL(clicked()), SLOT(onClickAcceptStart()));

  motion_icon = new CustomIconLabel(
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()));
  motion_label_ptr_ = new QLabel("Motion | Unknown");
  motion_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);
  horizontal_layout->setAlignment(Qt::AlignLeft);

  horizontal_layout->addWidget(motion_icon);
  horizontal_layout->addWidget(motion_label_ptr_);

  custom_container->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);
  custom_container->getLayout()->addWidget(accept_start_button_ptr_, 0, 2, 1, 4, Qt::AlignRight);

  custom_container->setContentsMargins(10, 0, 0, 0);

  group->addWidget(custom_container);

  return group;
}

QVBoxLayout * AutowareStatePanel::makeFailSafeGroup()
{
  auto * group = new QVBoxLayout;
  auto * v_layout = new QVBoxLayout;
  auto * custom_container1 = new CustomContainer(this);
  auto * custom_container2 = new CustomContainer(this);

  mrm_state_icon = new CustomIconLabel(
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()));
  mrm_behavior_icon = new CustomIconLabel(
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()));

  mrm_state_label_ptr_ = new QLabel("MRM State | Unknown");
  mrm_behavior_label_ptr_ = new QLabel("MRM Behavior | Unknown");

  // change text color
  mrm_state_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));
  mrm_behavior_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));

  auto * horizontal_layout = new QHBoxLayout;
  horizontal_layout->setSpacing(10);
  horizontal_layout->setContentsMargins(0, 0, 0, 0);

  horizontal_layout->addWidget(mrm_state_icon);
  horizontal_layout->addWidget(mrm_state_label_ptr_);

  custom_container1->getLayout()->addLayout(horizontal_layout, 0, 0, 1, 1, Qt::AlignLeft);

  auto * horizontal_layout2 = new QHBoxLayout;
  horizontal_layout2->setSpacing(10);
  horizontal_layout2->setContentsMargins(0, 0, 0, 0);

  horizontal_layout2->addWidget(mrm_behavior_icon);
  horizontal_layout2->addWidget(mrm_behavior_label_ptr_);

  custom_container2->getLayout()->addLayout(horizontal_layout2, 0, 0, 1, 1, Qt::AlignLeft);

  v_layout->addWidget(custom_container1);
  // v_layout->addSpacing(5);
  v_layout->addWidget(custom_container2);

  group->setContentsMargins(10, 0, 0, 0);

  group->addLayout(v_layout);
  return group;
}

/* QVBoxLayout * AutowareStatePanel::makeDiagnosticGroup()
{
  auto * group = new QVBoxLayout;

  // Create the scroll area
  QScrollArea * scrollArea = new QScrollArea;
  scrollArea->setFixedHeight(66);  // Adjust the height as needed
  scrollArea->setWidgetResizable(true);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Create a widget to contain the layout
  QWidget * scrollAreaWidgetContents = new QWidget;
  // use layout to contain the diagnostic label and the diagnostic level
  diagnostic_layout_ = new QVBoxLayout();
  diagnostic_layout_->setSpacing(5);                   // Set space between items
  diagnostic_layout_->setContentsMargins(5, 5, 5, 5);  // Set margins within the layout

  // Add a QLabel to display the title of what this is
  auto * tsm_label_title_ptr_ = new QLabel("Topic State Monitor: ");
  // Set the layout on the widget
  scrollAreaWidgetContents->setLayout(diagnostic_layout_);

  // Set the widget on the scroll area
  scrollArea->setWidget(scrollAreaWidgetContents);

  group->addWidget(tsm_label_title_ptr_);
  group->addWidget(scrollArea);

  return group;
} */

QVBoxLayout * AutowareStatePanel::makeVelocityLimitGroup()
{
  // Velocity Limit
  velocity_limit_setter_ptr_ = new QLabel("Set Velocity Limit");
  // set its width to fit the text
  velocity_limit_setter_ptr_->setFixedWidth(
    velocity_limit_setter_ptr_->fontMetrics().horizontalAdvance("Set Velocity Limit"));

  velocity_limit_value_label_ = new QLabel("0");
  velocity_limit_value_label_->setMaximumWidth(
    velocity_limit_value_label_->fontMetrics().horizontalAdvance("0"));

  CustomSlider * pub_velocity_limit_slider_ = new CustomSlider(Qt::Horizontal);
  pub_velocity_limit_slider_->setRange(0, 100);
  pub_velocity_limit_slider_->setValue(0);

  connect(pub_velocity_limit_slider_, &QSlider::sliderPressed, this, [this]() {
    sliderIsDragging = true;  // User starts dragging the handle
  });

  connect(pub_velocity_limit_slider_, &QSlider::sliderReleased, this, [this]() {
    sliderIsDragging = false;  // User finished dragging
    onClickVelocityLimit();    // Call when handle is released after dragging
  });

  connect(pub_velocity_limit_slider_, &QSlider::valueChanged, this, [this](int value) {
    this->velocity_limit_value_label_->setText(QString::number(value));
    velocity_limit_value_label_->setMaximumWidth(
      velocity_limit_value_label_->fontMetrics().horizontalAdvance(QString::number(value)));
    if (!sliderIsDragging) {   // If the value changed without dragging, it's a click on the track
      onClickVelocityLimit();  // Call the function immediately since it's not a drag operation
    }
  });

  // Emergency Button
  emergency_button_ptr_ = new CustomElevatedButton("Set Emergency");

  emergency_button_ptr_->setCursor(Qt::PointingHandCursor);
  // set fixed width to fit the text
  connect(emergency_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickEmergencyButton()));

  // Setting Multiple Goal Pose Button
  setting_multiple_goal_pose_button_ptr_ = new CustomElevatedButton("Setting Multiple Goal Pose");
  setting_multiple_goal_pose_button_ptr_->setCursor(Qt::PointingHandCursor);
  connect(setting_multiple_goal_pose_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickSettingMultipleGoalPose()));
  
  // Pose counter display
  pose_count_label_ptr_ = new QLabel("Goal Poses: 0");
  pose_count_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold; font-size: 14px; padding: 5px; background-color: %2; border-radius: 5px;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_primary_container.c_str())
      .arg(autoware::state_rviz_plugin::colors::default_colors.primary_container.c_str()));
  pose_count_label_ptr_->setAlignment(Qt::AlignCenter);
  pose_count_label_ptr_->setVisible(false); // Initially hidden
  
  // Apply initial styling for the new button
  setting_multiple_goal_pose_button_ptr_->updateStyle(
    "Setting Multiple Goal Pose",
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));

  // Multiple Goal Pose Control Buttons (initially hidden)
  finish_goal_pose_button_ptr_ = new CustomElevatedButton("Finish");
  finish_goal_pose_button_ptr_->setCursor(Qt::PointingHandCursor);
  finish_goal_pose_button_ptr_->setVisible(false);
  connect(finish_goal_pose_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickFinishMultipleGoalPose()));

  remove_last_goal_pose_button_ptr_ = new CustomElevatedButton("Remove Last");
  remove_last_goal_pose_button_ptr_->setCursor(Qt::PointingHandCursor);
  remove_last_goal_pose_button_ptr_->setVisible(false);
  connect(remove_last_goal_pose_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickRemoveLastGoalPose()));

  remove_all_goal_poses_button_ptr_ = new CustomElevatedButton("Remove All");
  remove_all_goal_poses_button_ptr_->setCursor(Qt::PointingHandCursor);
  remove_all_goal_poses_button_ptr_->setVisible(false);
  connect(remove_all_goal_poses_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickRemoveAllGoalPoses()));

  go_to_next_pose_button_ptr_ = new CustomElevatedButton("Go to Next Pose");
  go_to_next_pose_button_ptr_->setCursor(Qt::PointingHandCursor);
  go_to_next_pose_button_ptr_->setVisible(false);
  connect(go_to_next_pose_button_ptr_, SIGNAL(clicked()), this, SLOT(onClickGoToNextPose()));

  // Apply styling to control buttons
  finish_goal_pose_button_ptr_->updateStyle(
    "Finish",
    QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));

  remove_last_goal_pose_button_ptr_->updateStyle(
    "Remove Last",
    QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));

  remove_all_goal_poses_button_ptr_->updateStyle(
    "Remove All",
    QColor(autoware::state_rviz_plugin::colors::default_colors.error.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));

  go_to_next_pose_button_ptr_->updateStyle(
    "Go to Next Pose",
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));

  auto * utility_layout = new QVBoxLayout;
  auto * velocity_limit_layout = new QHBoxLayout;
  auto * velocity_limit_label = new QLabel("km/h");

  QFontMetrics fm(velocity_limit_value_label_->font());
  int width = fm.horizontalAdvance("999");

  // Set the fixed width for the label
  velocity_limit_value_label_->setFixedWidth(width);

  velocity_limit_layout->addWidget(pub_velocity_limit_slider_);
  velocity_limit_layout->addSpacing(5);
  velocity_limit_layout->addWidget(velocity_limit_value_label_);
  velocity_limit_layout->addWidget(velocity_limit_label);

  // Velocity Limit layout
  utility_layout->addSpacing(15);
  utility_layout->addWidget(velocity_limit_setter_ptr_);
  utility_layout->addSpacing(10);
  utility_layout->addLayout(velocity_limit_layout);
  utility_layout->addSpacing(25);
  utility_layout->addWidget(emergency_button_ptr_);
  utility_layout->addSpacing(10);
  utility_layout->addWidget(setting_multiple_goal_pose_button_ptr_);
  utility_layout->addSpacing(10);
  utility_layout->addWidget(pose_count_label_ptr_);
  
  // Multiple Goal Pose Control Buttons (initially hidden)
  utility_layout->addSpacing(5);
  utility_layout->addWidget(finish_goal_pose_button_ptr_);
  utility_layout->addSpacing(5);
  utility_layout->addWidget(remove_last_goal_pose_button_ptr_);
  utility_layout->addSpacing(5);
  utility_layout->addWidget(remove_all_goal_poses_button_ptr_);
  utility_layout->addSpacing(5);
  utility_layout->addWidget(go_to_next_pose_button_ptr_);
  
  // Add individual pose buttons layout
  utility_layout->addSpacing(15);
  pose_buttons_label_ = new QLabel("Individual Goal Poses:");
  pose_buttons_label_->setStyleSheet(
    QString("color: %1; font-weight: bold;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str()));
  pose_buttons_label_->setVisible(false); // Initially hidden
  utility_layout->addWidget(pose_buttons_label_);
  utility_layout->addSpacing(5);
  
  pose_buttons_layout_ = new QVBoxLayout;
  utility_layout->addLayout(pose_buttons_layout_);

  utility_layout->setContentsMargins(15, 0, 15, 0);

  return utility_layout;
}

void AutowareStatePanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  auto updateButtonState = [](
                             CustomSegmentedButtonItem * button, bool is_available,
                             uint8_t current_mode, uint8_t desired_mode, bool disable) {
    bool is_checked = (current_mode == desired_mode);
    button->setHovered(false);

    button->setActivated(is_checked);
    button->setChecked(is_checked);
    button->setDisabledButton(disable || !is_available);
    button->setCheckableButton(!disable && is_available && !is_checked);
  };

  bool disable_buttons = msg->is_in_transition;

  updateButtonState(
    auto_button_ptr_, msg->is_autonomous_mode_available, msg->mode, OperationModeState::AUTONOMOUS,
    disable_buttons);
  updateButtonState(
    stop_button_ptr_, msg->is_stop_mode_available, msg->mode, OperationModeState::STOP,
    disable_buttons);
  updateButtonState(
    local_button_ptr_, msg->is_local_mode_available, msg->mode, OperationModeState::LOCAL,
    disable_buttons);
  updateButtonState(
    remote_button_ptr_, msg->is_remote_mode_available, msg->mode, OperationModeState::REMOTE,
    disable_buttons);

  // toggle switch for control mode
  auto changeToggleSwitchState = [](CustomToggleSwitch * toggle_switch, const bool is_enabled) {
    // Flick the switch without triggering its function
    bool old_state = toggle_switch->blockSignals(true);
    toggle_switch->setCheckedState(!is_enabled);
    toggle_switch->blockSignals(old_state);
  };

  if (!msg->is_in_transition) {
    // would cause an on/off/on flicker if in transition
    changeToggleSwitchState(control_mode_switch_ptr_, !msg->is_autoware_control_enabled);
  }
}

void AutowareStatePanel::onRoute(const RouteState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;
  QString route_state = "Routing | Unknown";

  switch (msg->state) {
    case RouteState::UNSET:
      state = Pending;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str());
      route_state = "Routing | Unset";
      break;

    case RouteState::SET:
      state = Active;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str());
      route_state = "Routing | Set";
      break;

    case RouteState::ARRIVED:
      state = Danger;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.danger.c_str());
      route_state = "Routing | Arrived";
      break;

    case RouteState::CHANGING:
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str());
      state = Pending;
      route_state = "Routing | Changing";
      break;

    default:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      break;
  }

  routing_icon->updateStyle(state, bgColor);
  routing_label_ptr_->setText(route_state);

  clear_route_button_ptr_->updateStyle(
    "Clear Route",
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
  if (msg->state == RouteState::SET) {
    activateButton(clear_route_button_ptr_);
  } else {
    deactivateButton(clear_route_button_ptr_);
  }
}

void AutowareStatePanel::onLocalization(const LocalizationInitializationState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;
  QString localization_state = "Localization | Unknown";

  switch (msg->state) {
    case LocalizationInitializationState::UNINITIALIZED:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      localization_state = "Localization | Uninitialized";
      break;

    case LocalizationInitializationState::INITIALIZED:
      state = Active;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str());
      localization_state = "Localization | Initialized";
      break;

    case LocalizationInitializationState::INITIALIZING:
      state = Pending;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str());
      localization_state = "Localization | Initializing";
      break;

    default:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      break;
  }

  localization_icon->updateStyle(state, bgColor);
  localization_label_ptr_->setText(localization_state);
  init_by_gnss_button_ptr_->updateStyle(
    "Initialize with GNSS",
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
}

void AutowareStatePanel::onMotion(const MotionState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;
  QString motion_state = "Motion | Unknown";

  switch (msg->state) {
    case MotionState::STARTING:
      state = Pending;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str());
      motion_state = "Motion | Starting";
      break;

    case MotionState::MOVING:
      state = Active;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str());
      motion_state = "Motion | Moving";
      break;

    case MotionState::STOPPED:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.danger.c_str());
      motion_state = "Motion | Stopped";
      break;

    default:
      state = Danger;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      break;
  }

  motion_icon->updateStyle(state, bgColor);
  motion_label_ptr_->setText(motion_state);

  accept_start_button_ptr_->updateStyle(
    "Accept Start",
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(
      autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
  if (msg->state == MotionState::STARTING) {
    activateButton(accept_start_button_ptr_);
  } else {
    deactivateButton(accept_start_button_ptr_);
  }
}

void AutowareStatePanel::onMRMState(const MRMState::ConstSharedPtr msg)
{
  IconState state;
  QColor bgColor;
  QString mrm_state = "MRM State | Unknown";

  switch (msg->state) {
    case MRMState::NONE:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      mrm_state = "MRM State | Inactive";
      break;

    case MRMState::MRM_OPERATING:
      state = Active;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      mrm_state = "MRM State | Operating";
      break;

    case MRMState::MRM_SUCCEEDED:
      state = Active;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str());
      mrm_state = "MRM State | Successful";
      break;

    case MRMState::MRM_FAILED:
      state = Danger;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.danger.c_str());
      mrm_state = "MRM State | Failed";
      break;

    default:
      state = None;
      bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
      mrm_state = "MRM State | Unknown";
      break;
  }

  mrm_state_icon->updateStyle(state, bgColor);
  mrm_state_label_ptr_->setText(mrm_state);

  // behavior
  {
    IconState behavior_state;
    QColor behavior_bgColor;
    QString mrm_behavior = "MRM Behavior | Unknown";

    switch (msg->behavior) {
      case MRMState::NONE:
        behavior_state = Crash;
        behavior_bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
        mrm_behavior = "MRM Behavior | Inactive";
        break;

      case MRMState::PULL_OVER:
        behavior_state = Crash;
        behavior_bgColor =
          QColor(autoware::state_rviz_plugin::colors::default_colors.success.c_str());
        mrm_behavior = "MRM Behavior | Pull Over";
        break;

      case MRMState::COMFORTABLE_STOP:
        behavior_state = Crash;
        behavior_bgColor =
          QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str());
        mrm_behavior = "MRM Behavior | Comfortable Stop";
        break;

      case MRMState::EMERGENCY_STOP:
        behavior_state = Crash;
        behavior_bgColor =
          QColor(autoware::state_rviz_plugin::colors::default_colors.danger.c_str());
        mrm_behavior = "MRM Behavior | Emergency Stop";
        break;

      default:
        behavior_state = Crash;
        behavior_bgColor = QColor(autoware::state_rviz_plugin::colors::default_colors.info.c_str());
        mrm_behavior = "MRM Behavior | Unknown";
        break;
    }

    mrm_behavior_icon->updateStyle(behavior_state, behavior_bgColor);
    mrm_behavior_label_ptr_->setText(mrm_behavior);
  }
}

void AutowareStatePanel::onEmergencyStatus(
  const tier4_external_api_msgs::msg::Emergency::ConstSharedPtr msg)
{
  current_emergency_ = msg->emergency;
  if (msg->emergency) {
    emergency_button_ptr_->updateStyle(
      "Clear Emergency",
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.on_error_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_press.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_press.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_container.c_str()));
      //
  } else {
    emergency_button_ptr_->updateStyle(
      "Set Emergency", QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.error_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_tint.c_str()));
  }
}

void AutowareStatePanel::onSwitchStateChanged(int state)
{
  if (state == 0) {
    // call the control mode function
    onClickDirectControl();
  } else if (state == 2) {
    onClickAutowareControl();
  }
}

void AutowareStatePanel::onClickVelocityLimit()
{
  auto velocity_limit = std::make_shared<tier4_planning_msgs::msg::VelocityLimit>();
  velocity_limit->stamp = raw_node_->now();
  velocity_limit->max_velocity = velocity_limit_value_label_->text().toDouble() / 3.6;
  pub_velocity_limit_->publish(*velocity_limit);
}

void AutowareStatePanel::onClickAutonomous()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_autonomous_);
}
void AutowareStatePanel::onClickStop()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_stop_);
}
void AutowareStatePanel::onClickLocal()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_local_);
}
void AutowareStatePanel::onClickRemote()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_change_to_remote_);
}
void AutowareStatePanel::onClickAutowareControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_autoware_control_);
}
void AutowareStatePanel::onClickDirectControl()
{
  callServiceWithoutResponse<ChangeOperationMode>(client_enable_direct_control_);
}

void AutowareStatePanel::onClickClearRoute()
{
  callServiceWithoutResponse<ClearRoute>(client_clear_route_);
}

void AutowareStatePanel::onClickInitByGnss()
{
  callServiceWithoutResponse<InitializeLocalization>(client_init_by_gnss_);
}

void AutowareStatePanel::onClickAcceptStart()
{
  callServiceWithoutResponse<AcceptStart>(client_accept_start_);
}

void AutowareStatePanel::onClickEmergencyButton()
{
  using tier4_external_api_msgs::msg::ResponseStatus;
  using tier4_external_api_msgs::srv::SetEmergency;

  auto request = std::make_shared<SetEmergency::Request>();
  request->emergency = !current_emergency_;

  RCLCPP_INFO(raw_node_->get_logger(), request->emergency ? "Set Emergency" : "Clear Emergency");

  client_emergency_stop_->async_send_request(
    request, [this](rclcpp::Client<SetEmergency>::SharedFuture result) {
      const auto & response = result.get();
      if (response->status.code == ResponseStatus::SUCCESS) {
        RCLCPP_INFO(raw_node_->get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(
          raw_node_->get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}

void AutowareStatePanel::onClickSettingMultipleGoalPose()
{
  RCLCPP_INFO(raw_node_->get_logger(), "Setting Multiple Goal Pose button clicked");
  
  if (multiple_goal_pose_active_) {
    // If already active, stop the current session immediately without asking
    multiple_goal_pose_active_ = false;
    multiple_goal_pose_finished_ = false;
    multiple_goal_pose_from_csv_ = false;
    
    // Clear individual pose buttons when stopping
    clearIndividualPoseButtons();
    
    // Reset button to original state
    setting_multiple_goal_pose_button_ptr_->updateStyle(
      "Setting Multiple Goal Pose",
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
    
    RCLCPP_INFO(raw_node_->get_logger(), "Multiple goal pose collection stopped.");
    updateMultipleGoalPoseButtons();
    return;
  }
  
  // Ask user if they want to create new poses or load from CSV
  QMessageBox msgBox;
  msgBox.setWindowTitle("Goal Pose Setup");
  msgBox.setText("How would you like to set up goal poses?");
  msgBox.setIcon(QMessageBox::Question);
  
  QPushButton* newPosesBtn = msgBox.addButton("Set New Poses", QMessageBox::ActionRole);
  QPushButton* loadCsvBtn = msgBox.addButton("Load from CSV", QMessageBox::ActionRole);
  QPushButton* cancelBtn = msgBox.addButton(QMessageBox::Cancel);
  
  msgBox.exec();
  
  if (msgBox.clickedButton() == cancelBtn) {
    return;
  }
  
  if (msgBox.clickedButton() == loadCsvBtn) {
    // Load poses from CSV
    QString filename = QFileDialog::getOpenFileName(
      this,
      "Load Goal Poses",
      "",
      "CSV Files (*.csv)");
      
    if (!filename.isEmpty()) {
      if (loadPosesFromCSV(filename)) {
        QMessageBox::information(this, "Success", 
          QString("Loaded %1 goal poses from CSV file!").arg(goal_poses_.size()));
        RCLCPP_INFO(raw_node_->get_logger(), "Goal poses loaded from: %s", filename.toStdString().c_str());
        
        // Set to finished state for CSV-loaded poses so individual buttons appear
        multiple_goal_pose_active_ = false;
        multiple_goal_pose_finished_ = true;
        multiple_goal_pose_from_csv_ = true;
        current_goal_index_ = 0;
        // Reset to "no pose clicked yet" so first pose will be 1/n
        last_clicked_pose_index_ = SIZE_MAX;
        
        // Reset button to original state since poses are loaded and finished
        setting_multiple_goal_pose_button_ptr_->updateStyle(
          "Setting Multiple Goal Pose",
          QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
          QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
        
        // Show pose counter
        pose_count_label_ptr_->setVisible(true);
        updatePoseCountDisplay();
        updateMultipleGoalPoseButtons();
        return;
      } else {
        QMessageBox::warning(this, "Error", "Failed to load goal poses from file.");
        RCLCPP_ERROR(raw_node_->get_logger(), "Failed to load goal poses from: %s", filename.toStdString().c_str());
        return;
      }
    } else {
      return; // User cancelled file dialog
    }
  } else if (msgBox.clickedButton() == newPosesBtn) {
    // User chose "Set New Poses" - continue with the existing logic
    // Fall through to the existing code below
  } else {
    return; // Unknown button clicked
  }
  
  // If we reach here, user chose "Set New Poses"
  multiple_goal_pose_active_ = true;
  multiple_goal_pose_finished_ = false;
  multiple_goal_pose_from_csv_ = false;
  
    // Clear previous poses and reset index
    goal_poses_.clear();
    pose_names_.clear();
    current_goal_index_ = 0;
    last_clicked_pose_index_ = SIZE_MAX; // Reset to "no pose clicked yet"
    
    // Clear individual pose buttons
    clearIndividualPoseButtons();  // Show pose counter
  pose_count_label_ptr_->setVisible(true);
  updatePoseCountDisplay();
  
  // Update button text to indicate active state
  setting_multiple_goal_pose_button_ptr_->updateStyle(
    "Stop Setting Goals",
    QColor(autoware::state_rviz_plugin::colors::default_colors.warning.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
  
  RCLCPP_INFO(raw_node_->get_logger(), "Multiple goal pose collection started. Listening for poses on /planning/mission_planning/goal_t");
  
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::onClickFinishMultipleGoalPose()
{
  RCLCPP_INFO(raw_node_->get_logger(), "Finish Multiple Goal Pose button clicked");
  
  if (goal_poses_.empty()) {
    RCLCPP_WARN(raw_node_->get_logger(), "No goal poses to finish. Please add some poses first.");
    return;
  }
  
  // Ask user if they want to save poses to CSV
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Save Poses", 
    QString("Do you want to save the %1 goal poses to a CSV file?").arg(goal_poses_.size()),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::Yes);
    
  if (reply == QMessageBox::Yes) {
    QString filename = QFileDialog::getSaveFileName(
      this,
      "Save Goal Poses",
      QString("goal_poses_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")),
      "CSV Files (*.csv)");
      
    if (!filename.isEmpty()) {
      if (savePosesToCSV(filename)) {
        QMessageBox::information(this, "Success", "Goal poses saved successfully!");
        RCLCPP_INFO(raw_node_->get_logger(), "Goal poses saved to: %s", filename.toStdString().c_str());
      } else {
        QMessageBox::warning(this, "Error", "Failed to save goal poses to file.");
        RCLCPP_ERROR(raw_node_->get_logger(), "Failed to save goal poses to: %s", filename.toStdString().c_str());
      }
    }
  }
  
  multiple_goal_pose_active_ = false;
  multiple_goal_pose_finished_ = true;
  multiple_goal_pose_from_csv_ = false;
  current_goal_index_ = 0;
  // Reset to "no pose clicked yet" so first pose will be 1/n
  last_clicked_pose_index_ = SIZE_MAX;
  
  // Reset button to original state
  setting_multiple_goal_pose_button_ptr_->updateStyle(
    "Setting Multiple Goal Pose",
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_hover.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low_pressed.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
    QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
  
  RCLCPP_INFO(raw_node_->get_logger(), "Multiple goal poses finished. Total poses: %zu", goal_poses_.size());
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::onClickRemoveLastGoalPose()
{
  RCLCPP_INFO(raw_node_->get_logger(), "Remove Last Goal Pose button clicked");
  
  if (goal_poses_.empty()) {
    RCLCPP_WARN(raw_node_->get_logger(), "No goal poses to remove.");
    return;
  }
  
  goal_poses_.pop_back();
  pose_names_.pop_back();
  
  // Adjust last_clicked_pose_index_ if it points to the removed pose or becomes invalid
  if (goal_poses_.empty()) {
    last_clicked_pose_index_ = SIZE_MAX; // Reset to "no pose clicked yet"
  } else if (last_clicked_pose_index_ != SIZE_MAX && last_clicked_pose_index_ >= goal_poses_.size()) {
    last_clicked_pose_index_ = goal_poses_.size() - 1;
  }
  
  // Renumber remaining pose names to maintain continuity
  renumberPoseNames();
  RCLCPP_INFO(raw_node_->get_logger(), "Last goal pose removed. Remaining poses: %zu", goal_poses_.size());
  updatePoseCountDisplay();
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::onClickRemoveAllGoalPoses()
{
  RCLCPP_INFO(raw_node_->get_logger(), "Remove All Goal Poses button clicked");
  
  goal_poses_.clear();
  pose_names_.clear();
  current_goal_index_ = 0;
  last_clicked_pose_index_ = SIZE_MAX; // Reset to "no pose clicked yet"
  RCLCPP_INFO(raw_node_->get_logger(), "All goal poses removed.");
  updatePoseCountDisplay();
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::onClickGoToNextPose()
{
  RCLCPP_INFO(raw_node_->get_logger(), "Go to Next Pose button clicked");
  
  if (goal_poses_.empty()) {
    RCLCPP_WARN(raw_node_->get_logger(), "No goal poses available.");
    return;
  }
  
  // Calculate next pose index based on last clicked pose
  size_t next_pose_index;
  if (last_clicked_pose_index_ == SIZE_MAX) {
    // No pose clicked yet, start with first pose
    next_pose_index = 0;
  } else {
    // Normal case: next pose after last clicked
    next_pose_index = (last_clicked_pose_index_ + 1) >= goal_poses_.size() ? 
                     0 : last_clicked_pose_index_ + 1;
  }
  
  const auto & current_pose = goal_poses_[next_pose_index];
  std::string pose_name = (next_pose_index < pose_names_.size()) ? 
                         pose_names_[next_pose_index] : 
                         "pose_" + std::to_string(next_pose_index + 1);
  
  // Publish the PoseStamped directly
  pub_goal_pose_->publish(current_pose);
  
  RCLCPP_INFO(raw_node_->get_logger(), "Published goal pose '%s' (%zu/%zu)", 
             pose_name.c_str(), next_pose_index + 1, goal_poses_.size());
  
  // Update last clicked pose index to the one we just published
  last_clicked_pose_index_ = next_pose_index;
  
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::onGoalPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!multiple_goal_pose_active_ || multiple_goal_pose_finished_) {
    return;
  }
  
  // Don't add poses when using CSV-loaded poses (only collect when manually setting new poses)
  if (multiple_goal_pose_from_csv_) {
    return;
  }
  
  // Use the PoseStamped directly
  goal_poses_.push_back(*msg);
  
  // Generate a continuous name for the new pose
  std::string pose_name = "pose_" + std::to_string(goal_poses_.size());
  pose_names_.push_back(pose_name);
  
  RCLCPP_INFO(raw_node_->get_logger(), "Goal pose added from mission planning. Total poses: %zu", goal_poses_.size());
  updatePoseCountDisplay();
  updateMultipleGoalPoseButtons();
}

void AutowareStatePanel::updatePoseCountDisplay()
{
  QString count_text = QString("Goal Poses: %1").arg(goal_poses_.size());
  pose_count_label_ptr_->setText(count_text);
  
  // Change color based on pose count
  QString style_color;
  if (goal_poses_.size() == 0) {
    style_color = autoware::state_rviz_plugin::colors::default_colors.surface_variant.c_str();
  } else if (goal_poses_.size() < 5) {
    style_color = autoware::state_rviz_plugin::colors::default_colors.primary_container.c_str();
  } else {
    style_color = autoware::state_rviz_plugin::colors::default_colors.secondary_container.c_str();
  }
  
  pose_count_label_ptr_->setStyleSheet(
    QString("color: %1; font-weight: bold; font-size: 14px; padding: 5px; background-color: %2; border-radius: 5px;")
      .arg(autoware::state_rviz_plugin::colors::default_colors.on_primary_container.c_str())
      .arg(style_color));
}

void AutowareStatePanel::updateMultipleGoalPoseButtons()
{
  bool has_poses = !goal_poses_.empty();
  bool is_active = multiple_goal_pose_active_;
  bool is_finished = multiple_goal_pose_finished_;
  bool is_from_csv = multiple_goal_pose_from_csv_;
  
  // Show/hide control buttons based on state
  if (is_active && !is_finished) {
    if (is_from_csv) {
      // For CSV-loaded poses - only show navigation button
      finish_goal_pose_button_ptr_->setVisible(false);
      remove_last_goal_pose_button_ptr_->setVisible(false);
      remove_all_goal_poses_button_ptr_->setVisible(false);
      go_to_next_pose_button_ptr_->setVisible(has_poses);
    } else {
      // For manually collecting poses - show editing buttons
      finish_goal_pose_button_ptr_->setVisible(has_poses);
      remove_last_goal_pose_button_ptr_->setVisible(has_poses);
      remove_all_goal_poses_button_ptr_->setVisible(has_poses);
      go_to_next_pose_button_ptr_->setVisible(has_poses);
    }
  } else if (is_finished && has_poses) {
    // Finished state - only show navigation and removal controls
    finish_goal_pose_button_ptr_->setVisible(false);
    remove_last_goal_pose_button_ptr_->setVisible(false);
    remove_all_goal_poses_button_ptr_->setVisible(true);
    go_to_next_pose_button_ptr_->setVisible(true);
  } else {
    // No active session - hide all controls
    finish_goal_pose_button_ptr_->setVisible(false);
    remove_last_goal_pose_button_ptr_->setVisible(false);
    remove_all_goal_poses_button_ptr_->setVisible(false);
    go_to_next_pose_button_ptr_->setVisible(false);
  }
  
  // Show/hide pose counter based on state
  pose_count_label_ptr_->setVisible(is_active || (is_finished && has_poses));
  
  // Update button states
  if (has_poses && (is_active || is_finished)) {
    // Calculate next pose index based on last clicked pose
    // Special case: if no individual pose was clicked yet, show pose 1
    // Otherwise, show the pose after the last clicked one (with wrapping)
    size_t next_pose_index_0_based;
    if (last_clicked_pose_index_ == SIZE_MAX) {
      // No pose clicked yet, start with first pose
      next_pose_index_0_based = 0;
    } else {
      // Normal case: next pose after last clicked
      next_pose_index_0_based = (last_clicked_pose_index_ + 1) >= goal_poses_.size() ? 
                               0 : last_clicked_pose_index_ + 1;
    }
    size_t next_pose_index_1_based = next_pose_index_0_based + 1;
    
    QString button_text = QString("Go to Next Pose %1/%2")
                         .arg(next_pose_index_1_based)
                         .arg(goal_poses_.size());
    
    go_to_next_pose_button_ptr_->updateStyle(
      button_text,
      QColor(autoware::state_rviz_plugin::colors::default_colors.primary.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
  }
  
  // Enable/disable buttons based on state
  remove_last_goal_pose_button_ptr_->setEnabled(has_poses);
  remove_all_goal_poses_button_ptr_->setEnabled(has_poses);
  go_to_next_pose_button_ptr_->setEnabled(has_poses);
  
  // Update individual pose buttons
  updateIndividualPoseButtons();
}

bool AutowareStatePanel::savePosesToCSV(const QString & filename)
{
  try {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      RCLCPP_ERROR(raw_node_->get_logger(), "Could not open file for writing: %s", filename.toStdString().c_str());
      return false;
    }
    
    QTextStream out(&file);
    
    // Write CSV header
    out << "index,name,frame_id,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w,timestamp_sec,timestamp_nanosec\n";
    
    // Write each pose
    for (size_t i = 0; i < goal_poses_.size(); ++i) {
      const auto & pose = goal_poses_[i];
      // Get name from pose_names_ if available, otherwise use default naming
      QString name = (i < pose_names_.size() && !pose_names_[i].empty()) ? 
                     QString::fromStdString(pose_names_[i]) : 
                     QString("pose_%1").arg(i + 1);
      
      out << i + 1 << ","
          << name << ","
          << QString::fromStdString(pose.header.frame_id) << ","
          << pose.pose.position.x << ","
          << pose.pose.position.y << ","
          << pose.pose.position.z << ","
          << pose.pose.orientation.x << ","
          << pose.pose.orientation.y << ","
          << pose.pose.orientation.z << ","
          << pose.pose.orientation.w << ","
          << pose.header.stamp.sec << ","
          << pose.header.stamp.nanosec << "\n";
    }
    
    file.close();
    RCLCPP_INFO(raw_node_->get_logger(), "Successfully saved %zu poses to CSV", goal_poses_.size());
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(raw_node_->get_logger(), "Exception while saving CSV: %s", e.what());
    return false;
  }
}

bool AutowareStatePanel::loadPosesFromCSV(const QString & filename)
{
  try {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      RCLCPP_ERROR(raw_node_->get_logger(), "Could not open file for reading: %s", filename.toStdString().c_str());
      return false;
    }
    
    QTextStream in(&file);
    goal_poses_.clear();
    pose_names_.clear();
    
    // Skip header line
    if (!in.atEnd()) {
      QString header = in.readLine();
    }
    
    int line_number = 1;
    while (!in.atEnd()) {
      line_number++;
      QString line = in.readLine().trimmed();
      if (line.isEmpty()) continue;
      
      QStringList fields = line.split(',');
      // Support both old format (11 fields) and new format (12 fields with name)
      if (fields.size() != 11 && fields.size() != 12) {
        RCLCPP_WARN(raw_node_->get_logger(), "Invalid CSV format at line %d, expected 11 or 12 fields, got %d", 
                   line_number, fields.size());
        continue;
      }
      
      try {
        geometry_msgs::msg::PoseStamped pose;
        std::string pose_name;
        
        // Determine field indices based on whether name column exists
        int name_idx = (fields.size() == 12) ? 1 : -1;
        int frame_id_idx = (fields.size() == 12) ? 2 : 1;
        int pos_x_idx = (fields.size() == 12) ? 3 : 2;
        int pos_y_idx = (fields.size() == 12) ? 4 : 3;
        int pos_z_idx = (fields.size() == 12) ? 5 : 4;
        int ori_x_idx = (fields.size() == 12) ? 6 : 5;
        int ori_y_idx = (fields.size() == 12) ? 7 : 6;
        int ori_z_idx = (fields.size() == 12) ? 8 : 7;
        int ori_w_idx = (fields.size() == 12) ? 9 : 8;
        int stamp_sec_idx = (fields.size() == 12) ? 10 : 9;
        int stamp_nanosec_idx = (fields.size() == 12) ? 11 : 10;
        
        // Parse name if available
        if (name_idx >= 0) {
          pose_name = fields[name_idx].toStdString();
        } else {
          // Generate default name if not in CSV
          pose_name = "pose_" + std::to_string(pose_names_.size() + 1);
        }
        
        // Parse header
        pose.header.frame_id = fields[frame_id_idx].toStdString();
        pose.header.stamp.sec = fields[stamp_sec_idx].toInt();
        pose.header.stamp.nanosec = fields[stamp_nanosec_idx].toUInt();
        
        // Parse position
        pose.pose.position.x = fields[pos_x_idx].toDouble();
        pose.pose.position.y = fields[pos_y_idx].toDouble();
        pose.pose.position.z = fields[pos_z_idx].toDouble();
        
        // Parse orientation
        pose.pose.orientation.x = fields[ori_x_idx].toDouble();
        pose.pose.orientation.y = fields[ori_y_idx].toDouble();
        pose.pose.orientation.z = fields[ori_z_idx].toDouble();
        pose.pose.orientation.w = fields[ori_w_idx].toDouble();
        
        goal_poses_.push_back(pose);
        pose_names_.push_back(pose_name);
        
      } catch (const std::exception & e) {
        RCLCPP_WARN(raw_node_->get_logger(), "Error parsing line %d: %s", line_number, e.what());
        continue;
      }
    }
    
    file.close();
    
    // Only renumber poses that don't have names or have default names
    // Preserve custom names from CSV
    for (size_t i = 0; i < pose_names_.size(); ++i) {
      if (pose_names_[i].empty() || pose_names_[i].find("pose_") == 0) {
        pose_names_[i] = "pose_" + std::to_string(i + 1);
      }
    }
    
    RCLCPP_INFO(raw_node_->get_logger(), "Successfully loaded %zu poses from CSV", goal_poses_.size());
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(raw_node_->get_logger(), "Exception while loading CSV: %s", e.what());
    return false;
  }
}

void AutowareStatePanel::renumberPoseNames()
{
  for (size_t i = 0; i < pose_names_.size(); ++i) {
    pose_names_[i] = "pose_" + std::to_string(i + 1);
  }
}

void AutowareStatePanel::createIndividualPoseButtons()
{
  clearIndividualPoseButtons();
  
  for (size_t i = 0; i < goal_poses_.size(); ++i) {
    QString button_name = (i < pose_names_.size() && !pose_names_[i].empty()) ? 
                         QString::fromStdString(pose_names_[i]) : 
                         QString("pose_%1").arg(i + 1);
    
    CustomElevatedButton * pose_button = new CustomElevatedButton(button_name);
    pose_button->setCursor(Qt::PointingHandCursor);
    
    // Style the button
    pose_button->updateStyle(
      button_name,
      QColor(autoware::state_rviz_plugin::colors::default_colors.primary_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.on_primary_container.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.hover_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.pressed_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.checked_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_bg.c_str()),
      QColor(autoware::state_rviz_plugin::colors::default_colors.disabled_button_text.c_str()));
    
    // Connect button to slot with pose index
    connect(pose_button, &QPushButton::clicked, this, [this, i]() {
      onIndividualPoseButtonClicked(i);
    });
    
    individual_pose_buttons_.push_back(pose_button);
    pose_buttons_layout_->addWidget(pose_button);
    pose_buttons_layout_->addSpacing(3);
  }
}

void AutowareStatePanel::clearIndividualPoseButtons()
{
  // Delete all existing buttons and clear the layout
  for (auto * button : individual_pose_buttons_) {
    pose_buttons_layout_->removeWidget(button);
    delete button;
  }
  individual_pose_buttons_.clear();
  
  // Clear all items from layout
  QLayoutItem *child;
  while ((child = pose_buttons_layout_->takeAt(0)) != nullptr) {
    delete child;
  }
}

void AutowareStatePanel::updateIndividualPoseButtons()
{
  if (multiple_goal_pose_finished_ && !goal_poses_.empty()) {
    // Show label and create buttons when poses are finished
    pose_buttons_label_->setVisible(true);
    createIndividualPoseButtons();
  } else {
    // Hide label and clear buttons when not in finished state or no poses
    pose_buttons_label_->setVisible(false);
    clearIndividualPoseButtons();
  }
}

void AutowareStatePanel::onIndividualPoseButtonClicked(int pose_index)
{
  if (pose_index < 0 || pose_index >= static_cast<int>(goal_poses_.size())) {
    RCLCPP_WARN(raw_node_->get_logger(), "Invalid pose index: %d", pose_index);
    return;
  }
  
  // Update last clicked pose index for the "Go to Next Pose" button
  last_clicked_pose_index_ = static_cast<size_t>(pose_index);
  
  const auto & pose = goal_poses_[pose_index];
  std::string pose_name = (pose_index < static_cast<int>(pose_names_.size())) ? 
                         pose_names_[pose_index] : 
                         "pose_" + std::to_string(pose_index + 1);
  
  RCLCPP_INFO(raw_node_->get_logger(), "Publishing pose '%s' (index %d)", pose_name.c_str(), pose_index);
  
  // Publish the pose
  pub_goal_pose_->publish(pose);
  
  RCLCPP_INFO(raw_node_->get_logger(), "Successfully published pose '%s'", pose_name.c_str());
  
  // Update the "Go to Next Pose" button caption
  updateMultipleGoalPoseButtons();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareStatePanel, rviz_common::Panel)
