cmake_minimum_required(VERSION 3.8)
project(dr_clf_cbf_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CONTROL_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find ROS 2 interface dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 2. Generate the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeTwist.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# 3. Register the package (This is the most important part for Sourcing)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
