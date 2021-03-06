#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from robocon_msgs.msg import CCD_pose, Twist2DStamped, BoolStamped
from ccd_control.PID import PIDController
class ccd_control_node(object):
    def __init__(self):
        # Setup buffers
        self.d = 0.0
        self.phi = 0.0
        self.v = 0.0
        self.omega = 0.0

        # set state buffer
        self.active = True
        self.p_only = True
        if self.p_only:
            self.r = rospy.Rate(10)
        self.Kp = -0.1

        self.v_bar = 0.1

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped, queue_size=1)
        self.pub_car_cmd_sim = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_setpoint = rospy.Publisher("/setpoint", Float64, queue_size=1)
        self.pub_state = rospy.Publisher("/state", Float64, queue_size=1)

        # Setup subscriber
        self.sub_control = rospy.Subscriber("/control_effort", Float64, self.cbControl, queue_size=1)
        self.sub_ccd = rospy.Subscriber("/line_centroid", Float64, self.cbUpdate, queue_size=1)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data

    def cbUpdate(self, msg):
        if self.p_only:
            # Simple p control
            cur_state = float(msg.data)
            
            # image width:640, 320 is middle line
            control_effort = self.Kp * (cur_state - 320.0) / 50.0
            if abs(cur_state - 320.0) > 10:
                self.v_bar = 0.0
            else:
                self.v_bar = 0.1
            self.Pubsim(control_effort)
            self.r.sleep()
            return

        # use pid node    
        cur_state = Float64()
        cur_state.data = float(msg.data) / 100.0 - 3.2
        self.pub_state.publish(cur_state)
        cur_setpoint = Float64()
        cur_setpoint.data = 0.0
        self.pub_setpoint.publish(cur_setpoint)

    def cbControl(self, msg):
        control_effort_omega = msg.data
        self.Pubsim(control_effort_omega)

    def Pubsim(self, omega):
        if self.active == False:
            return
        # Publish control message into simulation
        car_cmd_msg = Twist()
        car_cmd_msg.linear.x = self.v_bar
        car_cmd_msg.angular.z = omega
        self.pub_car_cmd_sim.publish(car_cmd_msg)

    def Pub(self, omega):
        if self.active == False:
            return
        
        v = self.v_bar

        # Publish control message
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v_x = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_control_node', anonymous=False)

    # Create the NodeName object
    node = ccd_control_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()