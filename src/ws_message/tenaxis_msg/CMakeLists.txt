cmake_minimum_required(VERSION 3.8)
project(tenaxis_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 生成 .msg 接口所需
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成消息接口（把 msg 文件列在这里）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TenaxisImu.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 运行时导出（让其他包能依赖并 include 生成的头文件）
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
