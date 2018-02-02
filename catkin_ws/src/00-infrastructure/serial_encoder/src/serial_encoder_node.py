#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from serial_encoder.encoder import SerialEncoder as Encoder
from serial_encoder.translater import translate
from Tkinter import *
from robocon_msgs.msg import CCD_data, Twist2DStamped, FSMState, Joy6channel
import numpy as np

class serial_encoder_node(object):
    def __init__(self):
        # Init Decoder
        #root = Tk()
        #root.title("CCD Helper")
        self.encoder = Encoder()
        self.protocol = rospy.get_param("~serial_protocol")
        self.state = "JOYSTICK_CONTROL"

        # setup frequency
        self.frequency = 500.0
    
        # setup timer
        self.start_time = rospy.Time.now()

        self.select = 0
        self.ccd_msg = CCD_data()

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup subscriber
        self.sub_data = rospy.Subscriber("~joy_data", Joy6channel, self.cbJoyData)
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbState)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.01)
        # Create a timer that calls the process function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.frequency),self.cbprocess)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbState(self, msg):
        self.state = msg.JOYSTICK_CONTROL

    def cbJoyData(self,msg):
        # Check fsm state
        if self.state != "JOYSTICK_CONTROL":
            return

        # translate data
        data_list = translate(msg.channel_0,
                                   msg.channel_1,
                                   msg.channel_2,
                                   msg.channel_3,
                                   msg.button_0,
                                   msg.button_1)
        
        # Check protocol
        if len(data_list) != len(self.protocol):
            print "length data list: ", len(data_list)
            print "length protocol: ", len(self.protocol)
            raise Exception("Serial Protocol not matched!")

        # Write data to serial port
        rospy.loginfo("*************************************************")
        for i, datum in enumerate(data_list):
            self.encoder.write(datum)
            rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

    def translate(slef, msg):
        data_list = list()
        return

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('serial_encoder_node', anonymous=False)

    # Create the NodeName object
    node = serial_encoder_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()