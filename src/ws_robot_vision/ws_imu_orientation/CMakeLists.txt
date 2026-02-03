cmake_minimum_required(VERSION 3.8)
project(ws_imu_orientation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tenaxis_msg REQUIRED)

add_executable(imu_orientation_node src/imu_orientation_node.cpp)
ament_target_dependencies(imu_orientation_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tenaxis_msg
)

install(TARGETS imu_orientation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
