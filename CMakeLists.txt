cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_autonomy)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(PROGRAMS
   obstacle_avoidance.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(FILES
  launch/gazebo.launch
  launch/gazebo_world_large.launch
  launch/gazebo_world_small.launch
  worlds/location_test.world
  worlds/gaze_test_world_large.world
  worlds/gaze_test_world_small.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config launch maps worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)