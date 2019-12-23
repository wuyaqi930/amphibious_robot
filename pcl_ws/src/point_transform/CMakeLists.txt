cmake_minimum_required(VERSION 2.8.3)
project(point_transform)


find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

catkin_package(
    
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
  include ${OpenCV_INCLUDE_DIRS}
)


add_executable(point_transform
  src/point_transform.cpp
)


add_dependencies(point_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_transform
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
