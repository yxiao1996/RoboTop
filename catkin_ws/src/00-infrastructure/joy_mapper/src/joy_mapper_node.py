#!/usr/bin/env python
import rospy
#from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from robocon_msgs.msg import JoyAuto, JoyRemote, BoolStamped
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty

class JoyMapperNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.protocol_auto_flag = rospy.get_param("~protocol_auto_flag")
        self.cmd_auto = JoyAuto()
        self.cmd_remote = JoyRemote()

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist, queue_size=1)
        self.pub_joy_auto = rospy.Publisher("~joy_auto", JoyAuto, queue_size=1)
        self.pub_joy_remo = rospy.Publisher("~joy_remote", JoyRemote, queue_size=1)
        self.pub_buttons = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.cbJoy)
        # Setup service
        self.srv_joy = rospy.Service("~set_joy", Empty, self.cbSrvJoy)
        self.srv_joy_off = rospy.Service("~joy_off", Empty, self.cbSrvJoyOff)
        
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.1)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)
        
        self.v_gain = 100
        self.omega_gain = 10

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSrvJoy(self, req):
        # joystick override True button
        joy_override_msg = BoolStamped()
        joy_override_msg.header.stamp = rospy.Time.now()
        joy_override_msg.data = True
        self.pub_buttons.publish(joy_override_msg)
        return EmptyResponse()

    def cbSrvJoyOff(self, req):
        # joystick override True button
        joy_override_msg = BoolStamped()
        joy_override_msg.header.stamp = rospy.Time.now()
        joy_override_msg.data = False
        self.pub_buttons.publish(joy_override_msg)
        return EmptyResponse()

    def cbJoy(self,joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)
        #rospy.loginfo("[%s] %s" %(self.node_name,joy_msg.buttons))

    def publishControl(self):
        button_scalar = 0.4
        # Forward and Backward ax
        forward_ax = self.joy.axes[4]
        forward_bt = self.joy.axes[7]
        if abs(forward_ax) > 0.0 and abs(forward_bt) > 0.0:
            # Both ax and button push, not allowed
            forward_ctl = 0.0
        else:
            if abs(forward_ax) > 0.0:
                forward_ctl = forward_ax
            else:
                forward_ctl = forward_bt * button_scalar
        
        # Left and right ax
        left_ax = self.joy.axes[3]
        left_bt = self.joy.axes[6]
        if abs(left_ax) > 0.0 and abs(left_bt) > 0.0:
            # Both ax and button push, not allowed
            left_ctl = 0.0
        else:
            if abs(left_ax) > 0.0:
                left_ctl = left_ax
            else:
                left_ctl = left_bt * button_scalar

        # Rotation
        rot_ax = self.joy.axes[1]
        rot_l = self.joy.buttons[4]
        rot_r = self.joy.buttons[5]
        if rot_l == 1 and rot_r == 1:
            # Push both at same time, not allowed
            rot_bt = 0.0
        else:
            if rot_l == 1:
                rot_bt = 1.0
            else:
                if rot_r == 1:
                    rot_bt = -1.0
                else:
                    rot_bt = 0.0
        if abs(rot_ax) > 0.0 and abs(rot_bt) > 0.0:
            rot_ctl = 0.0
        else:
            if abs(rot_ax) > 0.0:
                rot_ctl = rot_ax
            else:
                rot_ctl = rot_bt * button_scalar

        # Generate joystick commands
        # check protocol type
        if self.protocol_auto_flag:
            joystick_cmd = JoyAuto()
            joystick_cmd.channel_0 = forward_ctl
            joystick_cmd.channel_1 = left_ctl
            joystick_cmd.channel_2 = rot_ctl
            joystick_cmd.channel_3 = -0.2
            joystick_cmd.channel_3 = 0.6
            joystick_cmd.channel_5 = 0.0
            joystick_cmd.channel_6 = 0.0
            joystick_cmd.channel_7 = float(self.joy.buttons[0])
            joystick_cmd.channel_8 = float(self.joy.buttons[1])
            self.cmd_auto = joystick_cmd
        else:
            joystick_cmd = JoyRemote()
            joystick_cmd.channel_0 = forward_ctl
            joystick_cmd.channel_1 = left_ctl
            joystick_cmd.channel_2 = rot_ctl
            joystick_cmd.channel_3 = 0.0
            joystick_cmd.button_0 = self.joy.buttons[0]
            joystick_cmd.button_1 = self.joy.buttons[1]
            self.cmd_remote = joystick_cmd

        #print joystick_cmd
        # Publish Joystick commands
        #self.pub_joy_auto.publish(joystick_cmd)

    def processButtons(self, joy_msg):
        # joystick override False button
        if joy_msg.buttons[7] == True:
            joy_override_msg = BoolStamped()
            joy_override_msg.header.stamp = rospy.Time.now()
            joy_override_msg.data = False
            self.pub_buttons.publish(joy_override_msg)

        # joystick override True button
        if joy_msg.buttons[6] == True:
            joy_override_msg = BoolStamped()
            joy_override_msg.header.stamp = rospy.Time.now()
            joy_override_msg.data = True
            self.pub_buttons.publish(joy_override_msg)

        return

    def cbTimer(self, _):
        # check protocol type
        if self.protocol_auto_flag:
            self.pub_joy_auto.publish(self.cmd_auto)
        else:
            self.pub_joy_remo.publish(self.cmd_remote)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('joy_mapper_node', anonymous=False)

    # Create the NodeName object
    node = JoyMapperNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
