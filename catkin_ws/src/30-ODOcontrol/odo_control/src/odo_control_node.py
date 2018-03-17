#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
from std_msgs.msg import String #Imports msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
"""
msg:
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

float64 x
float64 y
float64 theta
"""
class odo_control_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Initialize buffers
        self.x = 0
        self.y = 0
        self.theta = 0

        self.x_goal = 0
        self.y_goal = 0
        self.theta_goal = 0

        # Initialize constants
        self.theta_thresh = 0.1
        self.omega = 0.2
        self.v_bar = 0.6

        # Initialize node rate
        self.rate = rospy.Rate(5)

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        # Setup subscriber
        self.sub_goal = rospy.Subscriber("/goal", Pose2D, self.cbGoal, queue_size=1)
        self.sub_odometry = rospy.Subscriber("/odom", Odometry, self.cbOdom, queue_size=1)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbOdom(self,msg):
        # For a simple implementation, only take pose in account
        cur_pose = msg.pose.pose
        self.x = cur_pose.position.x
        self.y = cur_pose.position.y

        # Transform quaternion
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (row, pitch, yaw) = tf.transformations.euler_from_quaternion([q_x, q_y, q_z, q_w])
        self.theta = yaw

        # Simple navigation rule: turn to the correct direction, then run
        car_cmd = Twist()
        # Calculate the correct orientation
        delta_x = self.x_goal - self.x
        delta_y = self.y_goal - self.y
        delta_theta_cos = math.acos(delta_x / math.sqrt(delta_x**2 + delta_y**2))
        delta_theta_sin = math.asin(delta_y / math.sqrt(delta_x**2 + delta_y**2))
        delta_theta = np.angle(np.complex(delta_theta_cos, delta_theta_sin))

        print delta_x, delta_y, delta_theta, self.theta

        error_theta = delta_theta - self.theta
        #print error_theta
        # if head not point to goal, rotate
        if abs(error_theta) > self.theta_thresh:
            if error_theta < 0:
                car_cmd.angular.z = self.omega*(-1)
            else:
                car_cmd.angular.z = self.omega 
            car_cmd.linear.x = 0
            # Publish car command
            self.pub_car_cmd.publish(car_cmd)         
        else:
            if delta_x < 0:
                car_cmd.linear.x = -self.v_bar
            else:
                car_cmd.linear.x = self.v_bar
            # Publish cat command
            self.pub_car_cmd.publish(car_cmd)
        
        #self.rate.sleep()

    def cbGoal(self, msg):
        # Retrive goal from message
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('odo_control_node', anonymous=False)

    # Create the NodeName object
    node = odo_control_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()