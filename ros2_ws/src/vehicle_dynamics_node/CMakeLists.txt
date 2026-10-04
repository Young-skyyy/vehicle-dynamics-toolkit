cmake_minimum_required(VERSION 3.8)
project(vehicle_dynamics_node)

# C++17 for std::clamp, structured bindings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vehicle_msgs REQUIRED)

# Canonical native core is shared with the packaged FMU.
get_filename_component(VDT_NATIVE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../src/vehicle_dynamics_toolkit/native" ABSOLUTE)

# 编译 C++ 动力学节点
add_executable(dynamics_node src/vehicle_dynamics_node.cpp)
target_include_directories(dynamics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${VDT_NATIVE_DIR}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynamics_node
  rclcpp
  std_msgs
  vehicle_msgs
)

# Headless replay of the same header used by dynamics_node (no ROS in the core).
add_executable(dynamics_runner src/dynamics_runner.cpp)
target_include_directories(dynamics_runner PRIVATE include ${VDT_NATIVE_DIR})
install(DIRECTORY include/ DESTINATION include)
install(FILES ${VDT_NATIVE_DIR}/dynamics_core.hpp DESTINATION include/vehicle_dynamics_node)

# 安装可执行文件
install(TARGETS dynamics_node dynamics_runner
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 控制指令脚本
install(PROGRAMS
  scripts/control_pub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_pub
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
