/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//https://github.com/davetcoleman/moveit_dashboard/blob/kinetic-devel/src/moveit_panel.cpp

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QGroupBox>
#include <QSpinBox>

#include <sensor_msgs/msg/joy.hpp>

#include "welder_panel.hpp"
#include "rviz_common/display_context.hpp"


namespace welder_dashboard
{
WelderPanel::WelderPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  // Create a push button
  btn_home_ = new QPushButton(this);
  btn_home_->setText("Home");
  connect(btn_home_, SIGNAL(clicked()), this, SLOT(moveHome()));

  // Create a push button
  btn_scan_ = new QPushButton(this);
  btn_scan_->setText("Scan");
  connect(btn_scan_, SIGNAL(clicked()), this, SLOT(moveScan()));

  // Create a push button
  btn_add_obj_ = new QPushButton(this);
  btn_add_obj_->setText("Add collision objects");
  connect(btn_add_obj_, SIGNAL(clicked()), this, SLOT(addObjects()));

  // Create a push button
  btn_gen_path_ = new QPushButton(this);
  btn_gen_path_->setText("Generate path");
  connect(btn_gen_path_, SIGNAL(clicked()), this, SLOT(generatePath()));

  // Create a push button
  btn_p_and_ex_ = new QPushButton(this);
  btn_p_and_ex_->setText("Plan");
  connect(btn_p_and_ex_, SIGNAL(clicked()), this, SLOT(planAndExecute()));

  // Create a push button
  btn_ex_ = new QPushButton(this);
  btn_ex_->setText("Execute");
  connect(btn_ex_, SIGNAL(clicked()), this, SLOT(execute()));

  // Create a push button
  // btn_picture_ = new QPushButton(this);
  // btn_picture_->setText("Take a picture");
  // connect(btn_picture_, SIGNAL(clicked()), this, SLOT(takePicture()));

  // Create a push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop");
  btn_stop_->setStyleSheet("QPushButton { background-color: red; }");
  btn_stop_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(moveStop()));

  // Create remove last
  btn_remove_last_ = new QPushButton(this);
  btn_remove_last_->setText("Remove last");
  connect(btn_remove_last_, SIGNAL(clicked()), this, SLOT(removeLast()));

  // Create add to list
  btn_add_ = new QPushButton(this);
  btn_add_->setText("Add last");
  connect(btn_add_, SIGNAL(clicked()), this, SLOT(addLast()));


  // Create text view
  status_ = new QLabel(this);
  status_->setText("Push home to start.");

  // Horizontal Layout
  QHBoxLayout* hlayout1 = new QHBoxLayout;
  hlayout1->addWidget(btn_home_);
  hlayout1->addWidget(btn_scan_);
  hlayout1->addWidget(btn_add_obj_);
  hlayout1->addWidget(btn_gen_path_);
  hlayout1->addWidget(btn_add_);
  hlayout1->addWidget(btn_remove_last_);
  hlayout1->addWidget(btn_p_and_ex_);
  hlayout1->addWidget(btn_ex_);

  // Horizontal Layout
  QHBoxLayout* hlayout2 = new QHBoxLayout;
  // hlayout2->addWidget(new QLabel(QString("DEBUG:")));
  hlayout2->addWidget(status_);
  hlayout2->addWidget(btn_stop_);
  // hlayout2->addWidget(btn_picture_);
  // hlayout2->addWidget(spin_box_);
  // hlayout2->addWidget(combo_mode_);

  // Horizontal Layout
  // QHBoxLayout* hlayout3 = new QHBoxLayout;

  // this->setStyleSheet("QGroupBox {  border: 1px solid gray; padding-top: 0px; }");

  // Group box
  QGroupBox* group_box = new QGroupBox();
  group_box->setLayout(hlayout2);
  group_box->setFlat(false);

  // Verticle layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(hlayout1);
  layout->addWidget(group_box);
  // layout->addLayout( hlayout3 );
  setLayout(layout);


  // btn_home_->setEnabled(true);
  btn_scan_->setEnabled(false);
  btn_add_obj_->setEnabled(false);
  btn_gen_path_->setEnabled(false);
  btn_p_and_ex_->setEnabled(false);
  btn_ex_->setEnabled(false);
  btn_remove_last_->setEnabled(false);
  btn_add_->setEnabled(false);

  updateTopic();

  
}

void WelderPanel::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = std::make_shared<rclcpp::Node>("welderpanel");
  joy_publisher_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("panel_command", 10);
  //clock_ = raw_node->get_clock();
  status_subscriber_ = raw_node->template create_subscription<sensor_msgs::msg::Joy>("/program_status", 10, std::bind(&WelderPanel::statusCallback, this, std::placeholders::_1));
}

void WelderPanel::statusCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
  std::cout << msg->buttons[0] << std::endl;
  // switch (msg->buttons[0]){
  // case 1:
  //   std::cout << "Tootab" << std::endl;
  //   break;
  // default:
  //   break;
  // }
}

void WelderPanel::moveHome()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Move to home");
  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 1;
  joy_publisher_->publish(msg);
  status_->setText("Last step: HOME");
  btn_scan_->setEnabled(true);

  //rviz_common::DisplayContext* ctx = this->getDisplayContext();
  auto sel_manager = this->getDisplayContext()->getSelectionManager();

}

void WelderPanel::moveScan()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Start scanning");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 2;
  status_->setText("Last step: SCAN");
  joy_publisher_->publish(msg);
  btn_add_obj_->setEnabled(true);
}

void WelderPanel::addObjects()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Add collision objects to world");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 3;
  status_->setText("Last step: ADD COLLISION OBJECTS");
  btn_gen_path_->setEnabled(true);
  joy_publisher_->publish(msg);
}

void WelderPanel::generatePath()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Generate path from points");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 4;
  status_->setText("Last step: GENERATE PATH");
  joy_publisher_->publish(msg);

}

void WelderPanel::planAndExecute()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Plan");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 5;
  status_->setText("Last step: PLAN");
  joy_publisher_->publish(msg);
  btn_p_and_ex_->setEnabled(true);
}

void WelderPanel::takePicture()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Take picture");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 6;
  joy_publisher_->publish(msg);
}

void WelderPanel::moveStop()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Stopping");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 7;
  status_->setText("Last step: STOPPED");
  joy_publisher_->publish(msg);
}

void WelderPanel::removeLast()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Removing last weld");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 8;
  status_->setText("Last step: REMOVED LAST");
  joy_publisher_->publish(msg);
}

void WelderPanel::execute()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Execute");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 9;
  status_->setText("Last step: WELDING");
  joy_publisher_->publish(msg);
}

void WelderPanel::addLast()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Add last");

  sensor_msgs::msg::Joy msg;
  msg.buttons.resize(1);
  msg.buttons[0] = 10;
  status_->setText("Last step: ADDED PATH");
  joy_publisher_->publish(msg);
}

// void WelderPanel::homeRobot()
// {
//   RCLCPP_INFO_STREAM(rclcpp::get_logger("welder_panel_logger"), "Home Pose");

//   sensor_msgs::msg::Joy msg;
//   msg.buttons.resize(9);
//   msg.buttons[8] = 1;
//   joy_publisher_->publish(msg);
// }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void WelderPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}
// Load all configuration data for this panel from the given Config object.
void WelderPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}
}  // end namespace welder_panel

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(welder_dashboard::WelderPanel, rviz_common::Panel)
