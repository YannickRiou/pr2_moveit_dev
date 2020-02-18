cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	moveit_visual_tools
  roscpp
  rospy
  roslint
  geometry_msgs
  std_msgs
  actionlib
  actionlib_msgs
	tf2_ros
	tf2_geometry_msgs
)
find_package(Boost REQUIRED)

add_action_files(
  DIRECTORY action
  FILES
  move.action
  pick.action
  place.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 moveit_msgs
 actionlib_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_pickplace
#  CATKIN_DEPENDS sensor_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS actionlib_msgs tf2_ros tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(pr2_manip src/pr2_manip.cpp)
add_dependencies(pr2_manip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(pr2_manip ${catkin_LIBRARIES})

add_executable(pickClient src/pickClient.cpp)
add_dependencies(pickClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp )
target_link_libraries(pickClient ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
