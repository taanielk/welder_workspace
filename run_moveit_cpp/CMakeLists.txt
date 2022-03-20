
cmake_minimum_required(VERSION 3.5)
project(run_moveit_cpp)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

# find dependencies
find_package(ament_cmake REQUIRED)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# This shouldn't be necessary (required by moveit_simple_controller_manager)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2_eigen REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  rviz_visual_tools
  moveit_visual_tools
  moveit_ros_planning_interface
  interactive_markers
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  pcl_conversions
  PCL
)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(ur5_run_move_group src/run_moveit_cpp.cpp)
ament_target_dependencies(ur5_run_move_group
  moveit_ros_planning_interface
  Boost
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(ur5_run_move_group
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)

install(TARGETS ur5_run_move_group DESTINATION lib/${PROJECT_NAME})

install(TARGETS ur5_run_move_group
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
