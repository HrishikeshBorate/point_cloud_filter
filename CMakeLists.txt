# cmake_minimum_required(VERSION 3.0.2)
# project(point_cloud_filter)

# find_package(catkin REQUIRED COMPONENTS
#   pcl_conversions
#   pcl_ros
#   roscpp
#   sensor_msgs
# )

# ###################################
# ## catkin specific configuration ##
# ###################################
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES point_cloud_filter
# #  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )

# # Add the executable for point_cloud_filter_node
# add_executable(point_cloud_filter_node src/point_cloud_filter_node.cpp src/point_cloud_filter_common.cpp)
# target_link_libraries(point_cloud_filter_node ${catkin_LIBRARIES})

# # Add the executable for playback_controller_node
# add_executable(playback_controller_node src/playback_controller_node.cpp src/point_cloud_filter_common.cpp)
# target_link_libraries(playback_controller_node ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_filter)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_filter
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the executable for point_cloud_filter_node
# add_executable(point_cloud_filter_node src/point_cloud_filter_node.cpp src/point_cloud_filter_common.cpp)
add_executable(point_cloud_filter_node src/point_cloud_filter_node.cpp src/point_cloud_processing.cpp)
target_link_libraries(point_cloud_filter_node ${catkin_LIBRARIES})

