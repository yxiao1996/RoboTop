#!/usr/bin/env python
import rospy
import tf
# Imports message type
from std_msgs.msg import String 

# Define callback function
def callback(msg):
    s = "I heard: %s" % (msg.data)
    rospy.loginfo(s)

# Initialize the node with rospy
rospy.init_node('tf_listener_node', anonymous=False)

listener = tf.TransformListener()
rate = rospy.Rate(10.0)

# Create subscriber
subscriber = rospy.Subscriber("topic", String, callback)

while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/turtle1', '/carrot1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    print trans, rot
    rate.sleep()
