/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging MoveIt! applications
*/

// TODO: convert to flow layout:
// http://doc.qt.io/qt-5/qtwidgets-layouts-flowlayout-example.html

#ifndef WELDER_DASHBOARD__WELDER_PANEL_HPP
#define WELDER_DASHBOARD__WELDER_PANEL_HPP

#ifndef Q_MOC_RUN
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#endif

#include <sensor_msgs/msg/joy.hpp>
#include <QPushButton>
#include <QLabel>

class QLineEdit;
class QSpinBox;

namespace welder_dashboard
{
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// MoveItPanel will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class WelderPanel : public rviz_common::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  WelderPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  // Here we declare some internal slots.
protected Q_SLOTS:

  void moveHome();

  void moveScan();

  void addObjects();

  void moveStop();

  void planAndExecute();

  void takePicture();

  void generatePath();

  void homeRobot();

  void updateTopic();

  void removeLast();

  void execute();

  void addLast();

  void statusCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Then we finish up with protected member variables.
protected:
  QPushButton *btn_scan_;
  QPushButton *btn_add_obj_;
  QPushButton *btn_stop_;
  QPushButton *btn_p_and_ex_;
  QPushButton *btn_picture_;
  QPushButton *btn_home_;
  QPushButton *btn_gen_path_;
  QPushButton *btn_remove_last_;
  QPushButton *btn_ex_;
  QPushButton *btn_add_;

  QLabel *status_;


  // The ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr status_subscriber_;

};

}  // end namespace moveit_dashboard

#endif  // MOVEIT_DASHBOARD__MOVEIT_PANEL_H
