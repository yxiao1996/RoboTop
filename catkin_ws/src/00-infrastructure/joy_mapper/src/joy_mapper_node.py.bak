#!/usr/bin/env python
import rospy
#from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String, Bool #Imports msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from robocon_msgs.msg import Joy6channel, BoolStamped

class JoyMapperNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist, queue_size=1)
        self.pub_joystick = rospy.Publisher("~joy_data", Joy6channel, queue_size=1)
        self.pub_buttons = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.cbJoy)
        self.sub_virt = rospy.Subscriber("/virtual_joystick/cmd_vel", Twist, self.cbVirtJoy)
        self.sub_android_switch = rospy.Subscriber("/android/switch", Bool, self.cbAndSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.1)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        self.joy_msg = Joy6channel()
        self.v_gain = 100
        self.omega_gain = 10

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbAndSwitch(self, msg):
        # joystick override False button
        if msg.data == True:
            joy_override_msg = BoolStamped()
            joy_override_msg.header.stamp = rospy.Time.now()
            joy_override_msg.data = False
            self.pub_buttons.publish(joy_override_msg)

        # joystick override True button
        if msg.data == False:
            joy_override_msg = BoolStamped()
            joy_override_msg.header.stamp = rospy.Time.now()
            joy_override_msg.data = True
            self.pub_buttons.publish(joy_override_msg)

    def cbVirtJoy(self, msg):
        joy_msg = Joy()
        #joy_msg.axes[0] = 0.0
        #joy_msg.axes[1] = 0.0
        #joy_msg.axes[2] = 0.0
        joy_data = [0.0, 0.0, 0.0, msg.angular.z, msg.linear.x, 0.0, 0.0, 0.0]
        button_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #joy_msg.axes[4] = msg.linear.x
        #joy_msg.axes[3] = msg.angular.z
        joy_msg.axes = joy_data
        joy_msg.buttons = button_data
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)
        
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
        debug = False
        if debug:
            joystick_cmd = Joy6channel()
            joystick_cmd.channel_0 = self.joy.axes[4]
            joystick_cmd.channel_1 = self.joy.axes[3]
            joystick_cmd.channel_2 = self.joy.axes[1]
            joystick_cmd.channel_3 = -0.1
            joystick_cmd.channel_4 = 0.5
            joystick_cmd.channel_5 = 0.0
            joystick_cmd.channel_6 =  0.0
            joystick_cmd.channel_7 = float(self.joy.buttons[0])
            joystick_cmd.channel_8 = float(self.joy.buttons[1])

        joystick_cmd = Joy6channel()
        joystick_cmd.channel_0 = forward_ctl
        joystick_cmd.channel_1 = left_ctl
        joystick_cmd.channel_2 = rot_ctl
        joystick_cmd.channel_3 = -0.1
        joystick_cmd.channel_4 = 0.5
        joystick_cmd.channel_5 = 0.0
        joystick_cmd.channel_6 =  0.0
        joystick_cmd.channel_7 = self.joy.buttons[0]
        joystick_cmd.channel_8 = self.joy.buttons[1]

        self.joy_msg = joystick_cmd
        #print joystick_cmd
        # Publish Joystick commands
        #self.pub_joystick.publish(joystick_cmd)

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
        # Publish Joystick commands
        self.pub_joystick.publish(self.joy_msg)
        print self.joy_msg

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
