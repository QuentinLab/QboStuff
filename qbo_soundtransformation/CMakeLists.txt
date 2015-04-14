cmake_minimum_required(VERSION 2.8.3)
project(qbo_soundtransformation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qbo_soundtransformation
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(qbo_soundtransformation
#   src/${PROJECT_NAME}/qbo_soundtransformation.cpp
# )

## Declare a cpp executable
add_executable(qbo_soundtransformation src/qbo_soundtransformation.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(qbo_soundtransformation_node qbo_soundtransformation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(qbo_soundtransformation
   ${catkin_LIBRARIES}
 )

