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
)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
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
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include   
		    ${catkin_INCLUDE_DIRS}
		    ${Eigen_INCLUDE_DIRS})

add_executable(rl_utils src/robot_localization_utils.cpp)
add_dependencies(rl_utils robotnik_msgs_generate_messages_cpp)
target_link_libraries(rl_utils ${catkin_LIBRARIES})

install(
  TARGETS rl_utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
