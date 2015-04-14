cmake_minimum_required(VERSION 2.8.3)
project(qbo_face_xtion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #opencv2
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   FacesPos.msg 
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qbo_face_detection
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(/usr/include/opencv)
include_directories(
  ${catkin_INCLUDE_DIRS}
)




add_executable(qbo_face_xtion src/qbo_face_xtion.cpp)
#add_dependencies(qbo_face_detection qbo_face_detection_generate_messages_cpp)
 target_link_libraries(qbo_face_xtion
   ${catkin_LIBRARIES}
 )

add_dependencies(qbo_face_xtion qbo_face_xtion_generate_messages_cpp)
