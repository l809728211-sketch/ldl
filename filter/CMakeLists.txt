cmake_minimum_required(VERSION 3.5)
project(filter)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找并引入 ROS 2 组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PCL REQUIRED QUIET)
# 添加共享库目标
add_library(filter SHARED
  src/filter_node.cpp
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 链接依赖项

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(filter
  rclcpp
  sensor_msgs
  rclcpp_components
)

# 注册节点组件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN PointCloudProcessor
  EXECUTABLE filter_node
)

# 安装库和目标
install(TARGETS
  filter
  DESTINATION lib
)

# 导出依赖和信息
ament_export_dependencies(rclcpp_components)
ament_export_include_directories(include)
ament_export_libraries(filter)

ament_package()