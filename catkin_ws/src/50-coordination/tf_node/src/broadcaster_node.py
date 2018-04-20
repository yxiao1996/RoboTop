#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.5, 0.0, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "carrot1",
                         "turtle1")
        br.sendTransform((0.0, 0.5, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 1.57),
                         rospy.Time.now(),
                         "carrot2",
                         "turtle1")
        br.sendTransform((0.0, -0.5, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, -1.57),
                         rospy.Time.now(),
                         "carrot4",
                         "turtle1")
        br.sendTransform((-0.5, 0.0, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 3.14),
                         rospy.Time.now(),
                         "carrot3",
                         "turtle1")
        rate.sleep()