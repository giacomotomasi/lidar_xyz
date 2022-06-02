cmake_minimum_required(VERSION 3.0.2)
project(lidar_xyz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  rviz_visual_tools
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)


add_message_files(
  FILES
  ClustersArray.msg
  BoundingBox3D.msg
  BoundingBox3DArray.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(detector_xyz_node src/detector_xyz_node.cpp src/Detector_xyz.cpp)
target_link_libraries(detector_xyz_node ${catkin_LIBRARIES})

add_executable(bbox_xyz_node src/bbox_xyz_node.cpp src/Bbox_xyz.cpp)
target_link_libraries(bbox_xyz_node ${catkin_LIBRARIES})

add_executable(lidar_tf_broadcaster src/lidar_tf_broadcaster.cpp)
target_link_libraries(lidar_tf_broadcaster ${catkin_LIBRARIES})


