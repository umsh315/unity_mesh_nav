cmake_minimum_required(VERSION 3.5)
project(terrain_analysis)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加include目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# 创建cloud_interpolation库
add_library(cloud_interpolation STATIC
  src/cloud_interpolation.cpp
)

target_include_directories(cloud_interpolation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cloud_interpolation
  rclcpp
  pcl_ros
  pcl_conversions
)

# 主程序
add_executable(terrainAnalysis src/terrainAnalysis.cpp)
target_link_libraries(terrainAnalysis
  cloud_interpolation
)

ament_target_dependencies(terrainAnalysis 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  message_filters 
  pcl_ros 
  pcl_conversions
)

# 保持原有的可执行文件安装规则
install(TARGETS
  terrainAnalysis
  cloud_interpolation
  DESTINATION lib/${PROJECT_NAME}
)

# 单独安装库文件和头文件
install(TARGETS cloud_interpolation    # 指定要安装的目标(库)
  ARCHIVE DESTINATION lib    # 静态库(.a文件)的安装位置
  LIBRARY DESTINATION lib    # 动态库(.so文件)的安装位置
)

install(
  DIRECTORY include/         # 要复制的源目录， 表示要复制include目录下的所有内容
  DESTINATION include       # 目标安装位置
)

# 保持原有的launch文件安装规则
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
