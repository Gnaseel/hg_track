cmake_minimum_required(VERSION 3.0.2)
project(hg_lane)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  message_generation
  geometry_msgs
)

## Generate message in the 'msg' folder
# add_message_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion_project
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_Prefiltering src/Prefiltering.cpp)
add_dependencies(lidar_Prefiltering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_Prefiltering ${catkin_LIBRARIES})

# add_executable(get_lanePath src/get_lanePath.cpp)
# add_dependencies(get_lanePath ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(get_lanePath ${catkin_LIBRARIES})
