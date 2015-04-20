cmake_minimum_required(VERSION 2.8.3)
project(voronoi)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#  cmake_module
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
#  netpbm
  pcl_conversions
  pcl_ros
  pluginlib
  rosconsole
  roscpp
  std_msgs
  tf
  visualization_msgs
)

FIND_PACKAGE(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voronoi
#  CATKIN_DEPENDS cmake_module costmap_2d geometry_msgs nav_core nav_msgs netpbm pcl_conversions pcl_ros pluginlib rosconsole roscpp std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

include_directories(/usr/include/opencv)
include_directories(${catkin_INCLUDE_DIRS})

add_library(global_planner_lib src/testCostmap.cpp)

target_link_libraries(global_planner_lib
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

#add_executable(testCostmap_node src/testCostmap_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(voronoi_node voronoi_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(testCostmap_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   global_planner_lib
# )
