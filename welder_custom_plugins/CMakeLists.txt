cmake_minimum_required(VERSION 3.8)
project(welder_dashboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(welder_panel_headers_to_moc
  src/welder_panel.hpp
)
foreach(header "${welder_panel_headers_to_moc}")
  qt5_wrap_cpp(welder_panel_moc_files "${header}")
endforeach()

set(welder_panel_source_files
  src/welder_panel.cpp
)

add_library(welder_panel SHARED
  ${welder_panel_moc_files}
  ${welder_panel_source_files}
)

target_include_directories(welder_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(welder_panel PRIVATE "PUBLISH_SELECTION_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(welder_panel
  PUBLIC
  rclcpp
  rviz_common
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
)

install(
  TARGETS welder_panel
  EXPORT welder_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)



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