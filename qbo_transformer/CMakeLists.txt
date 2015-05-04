cmake_minimum_required(VERSION 2.8.3)
project(qbo_transformer)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  tf
  pcl_ros
)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  iNCLUDE_DIRS include
#  LIBRARIES qbo_transformer
#  CATKIN_DEPENDS roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(qbo_transformer
#   src/${PROJECT_NAME}/qbo_transformer.cpp
# )

## Declare a cpp executable
add_executable(qbo_transformer src/qbo_transformer.cpp)
## add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(qbo_transformer_node qbo_transformer_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(qbo_transformer
${catkin_LIBRARIES} 
${PCL_LIBRARIRES}
)

add_executable(mytransformer src/mytransformer.cpp)
target_link_libraries(mytransformer 
${catkin_LIBRARIES} 
${PCL_LIBRARIRES}
)
