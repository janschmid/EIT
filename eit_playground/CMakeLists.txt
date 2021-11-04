cmake_minimum_required(VERSION 2.8.3)
project(eit_playground)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eit_playground
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
include
${catkin_INCLUDE_DIRS}

)

add_compile_options(-pthread)

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})
add_dependencies(offb_node offb_node_cpp)

add_executable(offb_node_GPS src/offb_node_GPS.cpp)
target_link_libraries(offb_node_GPS ${catkin_LIBRARIES})
add_dependencies(offb_node_GPS offb_node_GPS_cpp)

add_executable(offb_node_guidance src/offb_node_guidance.cpp)
target_link_libraries(offb_node_guidance ${catkin_LIBRARIES})
add_dependencies(offb_node_guidance offb_node_guidance_cpp)

#add_executable(offb_node_control src/offb_node_control.cpp)
#target_link_libraries(offb_node_control ${catkin_LIBRARIES})
#add_dependencies(offb_node_control offb_node_control_cpp)

#############
## Install ##
#############
