cmake_minimum_required(VERSION 2.8.3)
project(robocon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # Robocon_msgs # Every Robocon packages should use this.
)


catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
