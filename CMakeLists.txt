cmake_minimum_required(VERSION 2.8.3)
project(nodelet_pcl_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  pcl_ros
  tf
  tf_conversions
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  geometric_shapes
  tf2_ros
  tf2
  tf2_geometry_msgs)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(
  FILES
  PointArray.msg
  ObjectArray.msg
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)


catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cluster_extractor src/cluster_extractor.cpp)
add_dependencies(cluster_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cluster_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(point_publisher src/point_publisher.cpp)
add_dependencies(point_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS src/plot_point_array.py src/table_cutoff_settings.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
