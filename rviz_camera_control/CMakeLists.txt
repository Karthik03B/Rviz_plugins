cmake_minimum_required(VERSION 3.8)
project(rviz_camera_control)

# --- Compiler setup -------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies -----------------------------------------------------------

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Qt MOC/UIC/RCC generation for headers containing Q_OBJECT.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# --- Message generation -----------------------------------------------------
#
# This package both defines and consumes its own interfaces. rosidl targets
# are namespaced with "__" suffixes (e.g. rviz_camera_control__rosidl_...),
# so there is no collision with the plugin library target defined below.

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraCommand.msg"
  "msg/CameraState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# --- Plugin library ----------------------------------------------------------
#
# Deliberately named differently from ${PROJECT_NAME} to keep it visually
# and mechanically distinct from the rosidl interface target group above.

set(PLUGIN_LIB_NAME "${PROJECT_NAME}_panel")

set(PLUGIN_HEADERS
  include/${PROJECT_NAME}/camera_control_panel.hpp
  include/${PROJECT_NAME}/external_view_controller.hpp
)

add_library(${PLUGIN_LIB_NAME} SHARED
  src/camera_control_panel.cpp
  src/external_view_controller.cpp
  ${PLUGIN_HEADERS}
)

target_include_directories(${PLUGIN_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PLUGIN_LIB_NAME}
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${PLUGIN_LIB_NAME} Qt5::Widgets)

# Link this package's own generated message typesupport into the plugin
# library. rosidl_get_typesupport_target() is the ament-blessed way to do
# this for a package that generates and consumes interfaces in one place
# (available on Foxy and newer, including Humble).
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PLUGIN_LIB_NAME} "${cpp_typesupport_target}")

target_compile_definitions(${PLUGIN_LIB_NAME} PRIVATE "RVIZ_CAMERA_CONTROL_BUILDING_LIBRARY")

# --- Install -----------------------------------------------------------------

install(
  TARGETS ${PLUGIN_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

# --- Testing -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
