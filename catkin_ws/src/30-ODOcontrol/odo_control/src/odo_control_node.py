#!/usr/bin/env python
import rospy
import math
import tf
import numpy as np
#from fuzzy_controllers import OmegaControl
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
from robocon_msgs.msg import BoolStamped, Pose2DStamped, Twist2DStamped
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
        self.debug_flag = True
        
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # set state buffer
        self.active = False
        self.trigger = False

        # Initialize buffers
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # Initialize constants
        self.theta_thresh = 3.0
        self.omega = 0.2
        self.v_bar = 0.6

        # Initialize node rate
        self.rate = rospy.Rate(50)

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_twist = rospy.Publisher("~twist2d", Twist2DStamped, queue_size=1)
        self.pub_reach = rospy.Publisher("~reach_goal", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_goal = rospy.Subscriber("~goal", Pose2DStamped, self.cbGoal, queue_size=1)
        self.sub_odometry = rospy.Subscriber("/odom", Odometry, self.cbOdom, queue_size=1)
        self.sub_pose = rospy.Subscriber("~pose", Pose2DStamped, self.cbPose, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.1)
        self.control_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data

    def cbOdom(self,msg):
        # Callback for simulation odometry
        if self.active == False:
            return

        debug = True

        if debug:
            rospy.loginfo("[%s] Hello!!" %(self.node_name))
            return
            
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

    def cbTimer(self, event):
        #print "odom control timer"
        if self.debug_flag == False:
            return
        # read robot state from parameter server
        robot_state = rospy.get_param("/robot_state")
        odom = robot_state[0]["odom"]
        pose_msg = Pose2DStamped()
        pose_msg.x = odom[0]
        pose_msg.y = odom[1]
        pose_msg.theta = odom[2]
        self.cbPose(pose_msg, True, True)

    def cbPose(self, msg, plot_pose=True, debug_flag=False):
        if debug_flag == False:
            return
        # Callback for pose message from decoder node
        if self.active == False:
            return
        if plot_pose:
            rospy.loginfo("[%s]: X: %s, Y: %s, Theta: %s" %(self.node_name, msg.x, msg.y, msg.theta))

        # Extract data from message
        time_stamp = msg.header.stamp
        x = msg.x
        y = msg.y
        theta = msg.theta
        
        # Some strange bug, need to  be fixed
        if x == 0.0 and y == 0.0 and theta == 0.0:
            return

        # Generate control effort
        # error
        error_x = x - self.x_goal
        error_y = y - self.y_goal
        #error_theta = theta - self.theta_goal
        comp_theta_goal = complex(np.cos(self.theta_goal * np.pi / 180.0), np.sin(self.theta_goal * np.pi / 180.0))
        comp_theta = complex(np.cos(theta * np.pi / 180.0), np.sin(theta * np.pi / 180.0))
        delta_theta = comp_theta / comp_theta_goal
        error_theta = np.angle(delta_theta) / np.pi * 180.0
        # Check at goal
        if abs(error_theta) < self.theta_thresh:
            if abs(error_x) < 100.0 and abs(error_y) < 100.0:
                reach_msg = BoolStamped()
                reach_msg.header.stamp = rospy.Time.now()
                reach_msg.data = True
                self.pub_reach.publish(reach_msg)
                self.active = False
        # for simplicity, use sign control, will add fuzzy controller
        sign_control = False
        if sign_control:
            if error_x > 0.0:
                v_x = -0.2
            else:
                if error_x < 0.0:
                    v_x = 0.2
                else:
                    v_x = 0.0
            
            if error_y > 0.0:
                v_y = -0.2
            else:
                if error_y < 0.0:
                    v_y = 0.2
                else:
                    v_y = 0.0
        sigmoid_p_control = True
        sigmoid_scalar = 2000.0
        dead_zone = 20.0
        if sigmoid_p_control:
            print "*"
            res_error_x = error_x / sigmoid_scalar
            res_error_y = error_y / sigmoid_scalar
            res_deadzone = 0.05
            res_distance = math.sqrt(res_error_x**2 + res_error_y**2)
            print res_error_x, res_error_y, res_distance
            position = complex(res_error_x, res_error_y)
            angle_map = np.angle(position, deg=True)
            angle_car = angle_map + theta
            print angle_car
            #trigger = False
            if res_distance < res_deadzone:
                v_x = 0.0
                v_y = 0.0
            else:
                #self.trigger = False
                control_effort = self.sigmoid(res_distance)
                v_x = -control_effort * np.cos(angle_car * np.pi / 180.0) / 1.0 #(res_error_x / res_distance)
                v_y = -control_effort * np.sin(angle_car * np.pi / 180.0) / 1.0#(res_error_y / res_distance)

        # Maintain the same frame with odometry
        simple_omega = False
        if simple_omega:
            if abs(theta-self.theta_goal) <self.theta_thresh:
                omega = 0.0
            else:
                #v_x = 0.0
                #v_y = 0.0
                if theta-self.theta_goal > 0:
                    omega = -0.2
                else:
                    omega = 0.2
        complex_omega = True
        if complex_omega:
            #comp_theta_goal = complex(np.cos(self.theta_goal * np.pi / 180.0), np.sin(self.theta_goal * np.pi / 180.0))
            #comp_theta = complex(np.cos(theta * np.pi / 180.0), np.sin(theta * np.pi / 180.0))
            #delta_theta = comp_theta / comp_theta_goal
            #error_theta = np.angle(delta_theta) / np.pi * 180.0
            rospy.loginfo("[%s] error theta [%s]"%(self.node_name, error_theta))
            if abs(error_theta) < self.theta_thresh:
                omega = 0.0
            elif error_theta > 0.0:
                omega = -0.2
            else:
                omega = 0.2

        # fuzzy controller
        use_fuzzy = False
        if use_fuzzy:
            if abs(theta) < self.theta_thresh:
                omega = 0.0
            else:
                v_x = 0.0
                v_y = 0.0
                fuzzy_system = OmegaControl()
                omega_ctl = fuzzy_system.getController()
                omega_ctl.input['theta_error'] = theta / 180.0
                omega_ctl.compute()
                omega = omega_ctl.output['out_omega']
                omega = omega / 2.0

        # Publish car command
        twist_msg = Twist2DStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.v_x = v_x
        twist_msg.v_y = v_y
        twist_msg.omega = omega
        self.pub_twist.publish(twist_msg)

    def cbGoal(self, msg):
        # Retrive goal from message
        if self.x_goal != msg.x or self.y_goal != msg.y or self.theta_goal != msg.theta:
            self.x_goal = msg.x
            self.y_goal = msg.y
            self.theta_goal = msg.theta
            self.trigger = False
            rospy.loginfo("[%s] Set current goal to (%s, %s, %s)" %(self.node_name, self.x_goal, self.y_goal, self.theta_goal))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('odo_control_node', anonymous=False)

    # Create the NodeName object
    node = odo_control_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
