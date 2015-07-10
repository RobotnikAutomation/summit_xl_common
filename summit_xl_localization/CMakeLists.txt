cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_localization
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  std_msgs
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES filter_base ekf
  CATKIN_DEPENDS
    robot_localization
    roscpp
    tf2
    tf2_ros
    message_filters
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include   
		    ${catkin_INCLUDE_DIRS}
		    ${Eigen_INCLUDE_DIRS})

# Library definition
add_library(navsat_transform_new src/navsat_transform_new.cpp)
# Dependencies
add_dependencies(navsat_transform_new robot_localization_gencpp)

# Executables
add_executable(navsat_transform_new_node src/navsat_transform_new_node.cpp)

# Linking
target_link_libraries(navsat_transform_new filter_utilities ros_filter_utilities ${catkin_LIBRARIES})
target_link_libraries(navsat_transform_new_node navsat_transform_new ${catkin_LIBRARIES})

#############
## Install ##
#############


