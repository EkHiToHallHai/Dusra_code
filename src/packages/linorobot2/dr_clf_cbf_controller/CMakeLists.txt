# cmake_minimum_required(VERSION 3.8)
# project(dr_clf_cbf_controller)

# find_package(ament_cmake REQUIRED)
# find_package(xacro REQUIRED)

# # # Install directories (config, launch, scripts)
# # install(DIRECTORY config scripts
# #   DESTINATION share/${PROJECT_NAME}
# # )

# # Install package.xml (THIS WAS MISSING)
# install(FILES package.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(dr_clf_cbf_controller)

# Default to C++14 (if you ever add C++ nodes)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (scripts)
install(PROGRAMS
  dr_clf_cbf_controller/dr_cbf_controller.py
  dr_clf_cbf_controller/dynamic_controller.py
  dr_clf_cbf_controller/utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATIONS share/${PROJECT_NAME}
)

ament_package()
