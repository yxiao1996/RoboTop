# Simulation Environment for Robocon 2018

* Using Gazebo simulator to build simulation, 
  aiming at tuning control algorithm in advance of mechanical engineering

## How to use this Simulation Environment

* add this directory to model path
    export GAZEBO_MODEL_PATH=${PWD}:${GAZEBO_MODEL_PATH}
    export GAZEBO_MODEL_PATH=`pwd`:$GAZEBO_MODEL_PATH
* launch world with robot
    roslaunch mybot_gazebo mybot_world.launch
* launch world without robot (modify world)
    roslaunch mybot_gazebo world.launch
