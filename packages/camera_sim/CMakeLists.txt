cmake_minimum_required(VERSION 3.0.2)
project(camera_sim)

# These must match the <build_depend> entries in package.xml.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Param files, so $(find camera_sim)/config/... resolves for roslaunch.
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Launch files, so $(find camera_sim)/launch/... resolves for roslaunch.
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# The sample images the node replays, found at runtime with rospkg.
install(DIRECTORY sample_images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sample_images
)
