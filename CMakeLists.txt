cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_autonomy)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()



install(FILES
  launch/gazebo.launch
  launch/gazebo_test_large.launch
  launch/gazebo_test_small.launch
  worlds/location_test.world
  worlds/gaze_test_large.world
  worlds/gaze_test_small.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config launch maps scripts worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)