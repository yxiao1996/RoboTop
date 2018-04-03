#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from dynamic_reconfigure.server import Server
from fuzzy_control.cfg import fuzzyConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {v0_lb}, {v0_ub}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("fuzzy_control", anonymous = True)

    srv = Server(fuzzyConfig, callback)
    rospy.spin()