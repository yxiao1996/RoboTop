#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from serial_encoder.encoder import SerialEncoder as Encoder
from serial_encoder.translater import translate, translateCTLtoJoy
from robocon_msgs.msg import CCD_data, Twist2DStamped, Joy6channel, BoolStamped
import numpy as np
import matplotlib.pyplot as plt 
import plot

class serial_encoder_node(object):
    def __init__(self):
        # Init Eecoder
        self.encoder = Encoder()
        self.protocol = rospy.get_param("~serial_protocol")
        self.joystick_state = True

        # setup frequency
        self.frequency = 5.0
        self.Rate = rospy.Rate(self.frequency)
        self.display_every = 3
        self.display_buffer = 0
    
        # display figure
        self.display_init = False
        #self.fig, self.axes = plt.subplots(6, 1)

        # setup timer
        self.start_time = rospy.Time.now()

        self.select = 0
        self.ccd_msg = CCD_data()

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup subscriber
        self.sub_joy_data = rospy.Subscriber("~joy_data", Joy6channel, self.cbJoyData, queue_size=1)
        self.sub_ctl_data = rospy.Subscriber("~ctl_data", Twist2DStamped, self.cbCtlData, queue_size=1)
        self.sub_mode = rospy.Subscriber("~switch", BoolStamped, self.cbState)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.01)
        # Create a timer that calls the process function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.frequency),self.cbDisplay)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbState(self, msg):
        self.joystick_state = msg.data

    def cbCtlData(self, msg):
        # check fsm state:
        if self.joystick_state == True:
            return
        
        # Translate data from Twisted2D to Joy6channel
        data_list = translateCTLtoJoy(msg.v_x,
                                      msg.v_y,
                                      msg.omega)
        
        # Check protocol
        if len(data_list) != 4:
            print "length data list: ", len(data_list)
            raise Exception("Serial Protocol not matched!")
        
        # Write data to serial port
        rospy.loginfo("*************************************************")
        for i, datum in enumerate(data_list):
            self.encoder.write(datum)
            rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))

    def cbJoyData(self,msg, plot_everything=True):
        # Check fsm state: joystick control
        if self.joystick_state != True:
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

        # Display
        if plot_everything:
            self.display(data_list)

        # Write data to serial port
        rospy.loginfo("*************************************************")        
        # Then send joystick data
        for i, datum in enumerate(data_list):
            self.encoder.write(datum)
            if self.display_buffer == self.display_every - 1:
                rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))

        self.display_buffer = (self.display_buffer + 1) % self.display_every
        # To meet the protocal send 12 zeros
        # for i in range(12):
        #     self.encoder.write(0)

        self.Rate.sleep()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

    def display(self, data_list):
        # plot everything using bars
        if self.display_init == False:
            self.fig, self.axes = plt.subplots(6, 1)
            self.display_init = True
        bars = []
        for datum in data_list:
            bar = plot.gen_bar(datum)
            bars.append(bar)
        plot.plot_bar(bars, self.axes)
        plt.pause(0.000001)

    def translate(self, msg):
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