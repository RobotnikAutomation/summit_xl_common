cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_localization
  tf2
  tf
  tf2_ros
  tf2_geometry_msgs
  message_filters
  std_msgs
  std_srvs
  geographic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  robotnik_msgs
  mavros_msgs
  robot_localization
  gmapping
  map_server
  amcl
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    robot_localization
    roscpp
    tf
    tf2
    tf2_ros
    message_filters
    std_msgs
    std_srvs
    geometry_msgs
    nav_msgs
    sensor_msgs
    robotnik_msgs
    mavros_msgs
)

###########
## Build ##
###########

install(
  DIRECTORY launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
