#!/bin/bash

# these things shouldn't be launched with sudo in the first place

killall rosmaster
killall gzserver
killall gzclient
roslaunch mybot_gazebo mybot_world.launch
