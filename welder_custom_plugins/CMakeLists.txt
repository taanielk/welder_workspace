cmake_minimum_required(VERSION 3.8)
project(welder_dashboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

##publish

find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

set(welder_panel_headers_to_moc
  src/welder_panel.hpp
)
set(selected_points_publisher_headers_to_moc
  src/SelectedPointsPublisher.h
)

foreach(header "${welder_panel_headers_to_moc}")
  qt5_wrap_cpp(welder_panel_moc_files "${header}")
endforeach()

foreach(header "${selected_points_publisher_headers_to_moc}")
  qt5_wrap_cpp(selected_points_publisher_moc_files "${header}")
endforeach()

set(welder_panel_source_files
  src/welder_panel.cpp
)

set(selected_points_publisher_source_files
  src/SelectedPointsPublisher.cpp
)

add_library(welder_panel SHARED
  ${welder_panel_moc_files}
  ${welder_panel_source_files}
)

add_library(selected_points_publisher SHARED
  ${selected_points_publisher_moc_files}
  ${selected_points_publisher_source_files}
)

target_include_directories(welder_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_include_directories(selected_points_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(selected_points_publisher PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ignition-math6
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(welder_panel PRIVATE "PUBLISH_SELECTION_BUILDING_LIBRARY")
target_compile_definitions(selected_points_publisher PRIVATE "selected_points_publisher_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(welder_panel
  PUBLIC
  rclcpp
  rviz_common
  sensor_msgs
)


ament_target_dependencies(selected_points_publisher
  PUBLIC
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
  pcl_conversions
  sensor_msgs
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(welder_panel HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rviz_common
  sensor_msgs
  ##publisher
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  resource_retriever
  rviz_ogre_vendor
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

install(
  TARGETS welder_panel
  EXPORT welder_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS selected_points_publisher
  EXPORT selected_points_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)



install(TARGETS selected_points_publisher DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
