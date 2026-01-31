cmake_minimum_required(VERSION 3.8)
project(ws_imu_jy901b)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tenaxis_msg REQUIRED)
find_package(Boost REQUIRED)
find_package(glog REQUIRED)

include_directories(include)

add_library(jy901b_driver
  src/jy901b_driver.cpp
)
ament_target_dependencies(jy901b_driver
  rclcpp std_msgs geometry_msgs tenaxis_msg
)
target_link_libraries(jy901b_driver Boost::boost glog::glog)

add_executable(jy901b_node src/jy901b_node.cpp)
ament_target_dependencies(jy901b_node
  rclcpp tenaxis_msg
)
target_link_libraries(jy901b_node jy901b_driver Boost::boost glog::glog)

install(TARGETS jy901b_driver jy901b_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
