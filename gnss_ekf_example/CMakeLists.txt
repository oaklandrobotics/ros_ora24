cmake_minimum_required(VERSION 3.5)
project(gnss_ekf)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/GnssEkf.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  gps_tools
  tf2_geometry_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::GnssEkf"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()