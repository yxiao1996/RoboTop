#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((-0.1, 0.0, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag0",
                         "ducky1")
        br.sendTransform((-0.1, 0.0, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "tag1",
                         "ducky1")
        br.sendTransform((0.0, -0.1, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 1.57),
                         rospy.Time.now(),
                         "tag2",
                         "ducky1")
        br.sendTransform((0.0, 0.1, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, -1.57),
                         rospy.Time.now(),
                         "tag4",
                         "ducky1")
        br.sendTransform((0.1, 0.0, 0.0),
                         #(0.0, 0.0, 0.0, 1.0),
                         tf.transformations.quaternion_from_euler(0, 0, 3.14),
                         rospy.Time.now(),
                         "tag3",
                         "ducky1")
        rate.sleep()