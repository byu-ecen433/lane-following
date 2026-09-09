cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

# These must match the <build_depend> entries in package.xml.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  duckietown_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs sensor_msgs duckietown_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Param files, so $(find lane_detection)/config/... resolves for roslaunch.
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
