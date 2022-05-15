////////////////////////////////////////////////////////////////////////////////
//
// Filename:      selected_points_topic.h
// Last change:   2013-11-21
// Authors:       Bartels, Philipp (mail@pBartels.net)
// Documentation: http://docs.ros.org/api/rviz/html/
// Version:       1.0.0
//
//////////////////////////////// DOCUMENTATION /////////////////////////////////
//
// Fork of the rviz::SelectionTool:
// Drag with the left button to select objects in the 3D scene.
// Hold the Alt key to change viewpoint as in the Move tool.
// Additionally publishes selected points on /selected_points topic.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////// CHANGELOG ///////////////////////////////////
//
// Version 1.0.0 (2013-11-21)
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO:
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SELECTED_POINTS_PUBLISHER_H
#define SELECTED_POINTS_PUBLISHER_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/tool.hpp"

# include <QCursor>
# include <QObject>
#endif

#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h>

namespace welder_panel
{

class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz_default_plugins::tools::SelectionTool
{
Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz_common::ViewportMouseEvent& event );

  virtual int processKeyEvent( QKeyEvent* event, rviz_common::RenderPanel* panel );

public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

  void PointCloudsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg);

protected:
  int _processSelectedAreaAndFindPoints();
  int _publishAccumulatedPoints();
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_selected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr real_selected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr partial_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bb_marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subs_;

  std::string tf_frame_;
  std::string rviz_cloud_topic_;
  std::string real_cloud_topic_;
  std::string subs_cloud_topic_;
  std::string bb_marker_topic_;
  bool selecting_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_segment_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_segment_pc_;

  pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract_indices_filter_;

  int num_acc_points_;
  int num_selected_points_;
};
} 
// end namespace welder_panel

#endif // SELECTED_POINTS_PUBLISHER_H

////////////////////////////////////////////////////////////////////////////////