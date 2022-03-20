#define BOOST_BIND_NO_PLACEHOLDERS
#include "run_moveit_cpp.hpp"
#include <math.h>       /* sin */
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
//PCL
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/io/obj_io.h>
#include <shape_msgs/msg/mesh.h>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>

// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI 3.14159265

using namespace std::chrono_literals;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_logger");
std::shared_ptr<rclcpp::Node> motion_planning_node;
pcl::PointCloud<pcl::PointXYZ>::Ptr current_camera_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_wo_normals (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr object_pointcloud (new pcl::PointCloud<pcl::PointNormal>); 
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanned_cloud_publisher;
sensor_msgs::msg::PointCloud2 output;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
geometry_msgs::msg::TransformStamped transformStamped;

std::vector<geometry_msgs::msg::Pose> calculate_circle_from_center(float centerX, float centerY, float z, float w, float radius){
  float x,y;
  std::vector<geometry_msgs::msg::Pose> circle_points;
  geometry_msgs::msg::Pose robot_pose;
  tf2::Quaternion q_orig, q_rot, q_new;
  for(int degree = 0; degree < 360; degree+=20){
    x = centerX + (radius * cos(degree*PI/180));
    y = centerY + (radius * sin(degree*PI/180));
    robot_pose.orientation.w = 1;
    robot_pose.position.x = x;
    robot_pose.position.y = y;
    robot_pose.position.z = z;
    circle_points.push_back(robot_pose);
  }
  return circle_points;
}

//https://github.com/team-vigir/vigir_perception/blob/master/vigir_point_cloud_proc/include/vigir_point_cloud_proc/mesh_conversions.h
static bool meshToShapeMsg(const pcl::PolygonMesh& in, shape_msgs::msg::Mesh& mesh)
{
  pcl_msgs::msg::PolygonMesh pcl_msg_mesh;

  pcl_conversions::fromPCL(in, pcl_msg_mesh);

  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t size = pcd_modifier.size();

  mesh.vertices.resize(size);

  std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";

  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

  for(size_t i = 0; i < size ; i++, ++pt_iter){
    mesh.vertices[i].x = pt_iter[0];
    mesh.vertices[i].y = pt_iter[1];
    mesh.vertices[i].z = pt_iter[2];
  }

  //ROS_INFO("Found %ld polygons", triangles.size());

  std::cout << "Updated vertices" << "\n";

  //BOOST_FOREACH(const Vertices polygon, triangles)

  mesh.triangles.resize(in.polygons.size());

  for (size_t i = 0; i < in.polygons.size(); ++i)
  {
    if(in.polygons[i].vertices.size() < 3)
    {
      //ROS_WARN("Not enough points in polygon. Ignoring it.");
      std::cout << "Not enough points in polygon. Ignoring it." << std::endl;
      continue;
    }

    //shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
    //boost::array<uint32_t, 3> xyz = {{in.polygons[i].vertices[0], in.polygons[i].vertices[1], in.polygons[i].vertices[2]}};
    //triangle.vertex_indices = xyz;

    //mesh.triangles.push_back(shape_msgs::MeshTriangle());
    //mesh.triangles[i].vertex_indices.resize(3);

    for (int j = 0; j < 3; ++j)
      mesh.triangles[i].vertex_indices[j] = in.polygons[i].vertices[j];
  }
  return true;
}

void add_mesh_to_movegroup()
{
  pcl::OrganizedFastMesh<pcl::PointXYZ>::Ptr orgMesh (new pcl::OrganizedFastMesh<pcl::PointXYZ>());
  pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);
  //pcl::PolygonMesh triangles ;
  // orgMesh.useDepthAsDistance(true);
  orgMesh->setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT  );
  orgMesh->setInputCloud(current_camera_cloud);
  orgMesh->reconstruct(*triangles);
  // qDebug("%i", triangles->polygons.size());
  pcl::io::saveOBJFile("filtered.obj", *triangles);

  shape_msgs::msg::Mesh shape_mesh;
  meshToShapeMsg(*triangles, shape_mesh);

  const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(motion_planning_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // geometry_msgs::msg::Pose mesh_pose;
  // mesh_pose.orientation.w = 1.0;
  // mesh_pose.position.x = 0.48;
  // mesh_pose.position.y = 0.0;
  // mesh_pose.position.z = 0.25;

  collision_object.id = "mesh";

  collision_object.meshes.push_back(shape_mesh);
  //collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // planning_scene_interface.addCollisionObjects(collision_objects);
  // Create search tree*
  // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  // tree2->setInputCloud(object_pointcloud);

  // // Initialize objects
  // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  // pcl::PolygonMesh triangles;

  // // Set the maximum distance between connected points (maximum edge length)

  // gp3.setSearchRadius(0.5); // Triangle Size
  // gp3.setMu(2.5); // 2.5 ~ 3.0
  // gp3.setMaximumNearestNeighbors(150); // 100 ~ 200
  // gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  // gp3.setMinimumAngle(M_PI / 18); // 10 degrees
  // gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
  // gp3.setNormalConsistency(true);
  // //gp3.setConsistentVertexOrdering(true);
  // std::cout << "tehtud1" << std::endl;

  // // Get result
  // gp3.setInputCloud(object_pointcloud);
  // std::cout << "tehtud2" << std::endl;

  // gp3.setSearchMethod(tree2);
  // std::cout << "tehtud3" << std::endl;

  // gp3.reconstruct(triangles);
  // std::cout << "tehtud4" << std::endl;

  // // Additional vertex information
  // std::vector<int> parts = gp3.getPartIDs();
  // std::vector<int> states = gp3.getPointStates();


  // std::cout << "tehtud4" << std::endl;
  // Finish
}

//https://answers.ros.org/question/358163/ros2-performance-issue-in-custom-transformpointcloud-function-compared-to-pcl_rostransformpointcloud-function/
void tpc (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, geometry_msgs::msg::TransformStamped& transform) {
  geometry_msgs::msg::Quaternion q = transform.transform.rotation;
  Eigen::Quaternionf rotation (q.w, q.x, q.y, q.z);
  geometry_msgs::msg::Vector3 v = transform.transform.translation; 
  Eigen::Vector3f origin (v.x, v.y, v.z);
  pcl::transformPointCloud(*cloud_in, *cloud_out, origin, rotation); 
}

void add_to_pointcloud(){
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_with_normals (new pcl::PointCloud<pcl::PointNormal>); 

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  transformStamped = tf_buffer_->lookupTransform("d435i_depth_optical_frame", "world",tf2::TimePointZero);
  tpc(current_camera_cloud,test_cloud, transformStamped);
  *object_pointcloud_wo_normals +=  *current_camera_cloud;
  ne.setInputCloud(current_camera_cloud);
  ne.compute(*normals);

  pcl::concatenateFields(*current_camera_cloud, *normals, *pointcloud_with_normals);

  *object_pointcloud += *pointcloud_with_normals;
}

void move_to_start(std::shared_ptr<rclcpp::Node> motion_node){
  const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(motion_node, PLANNING_GROUP);
  move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));
  move_group.move();
}

void scan_object(std::shared_ptr<rclcpp::Node> motion_node){
  const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(motion_node, PLANNING_GROUP);


  move_group.setEndEffectorLink("gripper_link");
  // geometry_msgs::msg::PoseStamped pos = move_group.getPoseTarget();

  // std::cout << pos.pose.position.x << std::endl;
  // Visualization
  // ^^^^^^^^^^^^^
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(motion_node, "base_link", "welding_demo_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();

  std::vector<geometry_msgs::msg::Pose> waypoints = calculate_circle_from_center(0.1, 0.2, 0.7, 0.5, 0.2);

  //std::cout << move_group.getCurrentRPY() << std::endl;

  move_group.setPoseTarget(waypoints.front());
  move_group.move();

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  std::cout << waypoints.size() << std::endl;
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  move_group.execute(trajectory);

  // for(geometry_msgs::msg::Pose ps : waypoints){
  //   move_group.setPoseTarget(ps);
  //   move_group.move();
  // }

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
}

class PanelCommandSubscriber : public rclcpp::Node
{
  public:
    PanelCommandSubscriber()
    : Node("panel_command_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/panel_command", 10, std::bind(&PanelCommandSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      *action_ptr = msg->buttons[0];
      std::cout << action_number << std::endl;
      switch (action_number){
        //move to home pos
        case 1:
          move_to_start(motion_planning_node);
          break;

        //move in circular motion aroudn center point (scan)
        case 2:
          scan_object(motion_planning_node);
          break;

        //generate objects into world
        case 3:
          add_mesh_to_movegroup();
          //pcl::toROSMsg(*object_pointcloud, output);
          //scanned_cloud_publisher->publish(output);
          break;

        //generate welding path and visualize with waypoints
        case 4:
          break;

        //plan and execute path
        case 5:
          break;

        //DEBUG: TAKE PICTURE
        case 6:
          add_to_pointcloud();
          break;

        //DEBUG: STOP
        case 7:
          break;

        default:
          break;
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

class CameraSubscriber : public rclcpp::Node
{
  public:
    CameraSubscriber()
    : Node("camera_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/d435i/depth/color/points", 10, std::bind(&CameraSubscriber::pointCloud2Callback, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        // RCLCPP_INFO(this->get_logger(), "I heard");
        // std::cout << msg->header.frame_id << std::endl;
        pcl::fromROSMsg(*msg, *current_camera_cloud);
        // std::cout << current_camera_cloud->height << std::endl;
        // std::cout << current_camera_cloud->width << std::endl;

        // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

        // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
        // ne.setMaxDepthChangeFactor(0.02f);
        // ne.setNormalSmoothingSize(10.0f);
        // ne.setInputCloud(current_camera_cloud);
        // ne.compute(*normals);

        // pcl::concatenateFields(*current_camera_cloud, *normals, *object_pointcloud);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

class ScannedCloudPublisher : public rclcpp::Node
{
  public:
    ScannedCloudPublisher()
    : Node("scanned_cloud_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scanned_cloud", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ScannedCloudPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*test_cloud, output);
      output.header.frame_id = "world"; //d435i_depth_optical_frame
      // std::cout << output.header.frame_id << std::endl;
      output.header.stamp = rclcpp::Clock().now();
      publisher_->publish(output);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  motion_planning_node = rclcpp::Node::make_shared("motion_planning_logger", node_options);

  auto node1 = std::make_shared<PanelCommandSubscriber>();
  auto camera_node = std::make_shared<CameraSubscriber>();
  auto scanned_cloud_publisher = std::make_shared<ScannedCloudPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(camera_node);
  executor.add_node(motion_planning_node);
  executor.add_node(node1);
  executor.add_node(scanned_cloud_publisher);
  // scanned_cloud_publisher = camera_node->template create_publisher<sensor_msgs::msg::PointCloud2>("scanned_cloud", 10);
  executor.spin();
  // std::thread([&executor]() { executor.spin(); }).detach();
  // geometry_msgs::msg::Pose target_pose1;
  // RCLCPP_INFO(motion_node->get_logger(), "CENTER ON OBJECT");
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 1.0;
  // target_pose1.position.y = 1.0;
  // target_pose1.position.z = 1.0;
  // move_group.setPoseTarget(target_pose1);
  rclcpp::shutdown();
  return 0;
}