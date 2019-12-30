cmake_minimum_required(VERSION 2.8.3)
project(point_transform)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
)
find_package(PCL 1.2 REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(point_transform
  src/point_transform.cpp
)

link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_dependencies(point_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_transform
  ${catkin_LIBRARIES}
  ${PCL_LIBS}
)
