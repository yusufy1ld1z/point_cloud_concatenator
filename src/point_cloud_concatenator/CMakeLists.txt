cmake_minimum_required(VERSION 3.5)
project(point_cloud_concatenator)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(pc_concatenate src/pointcloud_concatenate_node.cpp)
ament_target_dependencies(pc_concatenate
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  message_filters
)

# Install executable
install(TARGETS pc_concatenate
  DESTINATION lib/${PROJECT_NAME}
)

# # Install launch file
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# Install package.xml
# install(FILES package.xml
#   DESTINATION share/${PROJECT_NAME}
# )

# Export package dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  ${PCL_LIBRARIES}
)

# Install dependencies
ament_package()