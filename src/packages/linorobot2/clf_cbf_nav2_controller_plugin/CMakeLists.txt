cmake_minimum_required(VERSION 3.8)
project(clf_cbf_nav2_controller_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(dr_clf_cbf_controller REQUIRED)
find_package(dr_clf_cbf_interfaces REQUIRED)

add_library(clf_cbf_nav2_controller_plugin SHARED
  src/clf_cbf_controller.cpp
)

target_include_directories(clf_cbf_nav2_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(clf_cbf_nav2_controller_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
  dr_clf_cbf_interfaces
  # dr_clf_cbf_controller
)

install(TARGETS clf_cbf_nav2_controller_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

ament_package()
