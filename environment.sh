#!/bin/bash
#[ -z "$ROBOCON_ROOT" ] && { echo "Need to set ROBOCON_ROOT - configuration is invalid (!)";  }
[ -z "$HOSTNAME"        ] && { echo "Need to set HOSTNAME.";        }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/kinetic/setup.$shell

export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

export ROBOCON_ROOT=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
echo "Set ROBOCON_ROOT to: $ROBOCON_ROOT"

export PYTHONPATH=$ROBOCON_ROOT/catkin_ws/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

# Cannot make machines before building
# echo "Building machines file..."
# make -C $ROBOCON_ROOT machines

echo "Activating development environment..."
source $ROBOCON_ROOT/catkin_ws/devel/setup.$shell

if [ 2015 -ge $(date +%Y) ];          
then
    >&2 echo "Error! Time travel detected. System time is: $(date)"
fi

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
