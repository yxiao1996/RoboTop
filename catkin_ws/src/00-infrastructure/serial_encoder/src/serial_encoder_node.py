#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from serial_encoder.encoder import SerialEncoder as Encoder
from serial_encoder.translater import translateAuto, translateRemote, translateCTLtoAuto, translateCTLtoRemote, translateCTLtoMove
from robocon_msgs.msg import CCD_data, Twist2DStamped, JoyRemote, JoyAuto, BoolStamped
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
import numpy as np
import matplotlib.pyplot as plt 
import plot
from codebook import CodeBook 

class serial_encoder_node(object):
    def __init__(self):
        # Init Eecoder
        self.protocol_auto = rospy.get_param("~protocol_auto")
        self.protocol_remote = rospy.get_param("~protocol_remote")
        self.baudrate = rospy.get_param("~baudrate")
        
        connect_success = False
        while(connect_success==False):
            # try to connect
            try:
                self.encoder = Encoder(self.baudrate)
                connect_success = True
            except Exception as e:
                print e
                rospy.sleep(0.1)
        self.joystick_state = True

        # setup frequency
        self.frequency = rospy.get_param("~update_frequency")
        self.protocol_auto_flag = rospy.get_param("~protocol_auto_flag")
        self.Rate = rospy.Rate(self.frequency)
        self.display_every = 3
        self.display_buffer = 0
        if self.protocol_auto_flag:
            self.protocol = self.protocol_auto
        else:
            self.protocol = self.protocol_remote
        # display figure
        self.display_init = False
        #self.fig, self.axes = plt.subplots(6, 1)

        # setup timer
        self.start_time = rospy.Time.now()

        self.select = 0
        self.ccd_msg = CCD_data()

        # states
        self.button_0 = 0.0
        self.button_1 = 0.0
        self.fetch_state = 0.0
        self.codebook = CodeBook()

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        rospy.loginfo("[%s]: baudrate: [%s]" %(self.node_name, self.baudrate))

        # Setup subscriber
        self.sub_joy_auto = rospy.Subscriber("~joy_auto", JoyAuto, self.cbJoy, queue_size=1)
        self.sub_joy_remo = rospy.Subscriber("~joy_remote", JoyRemote, self.cbJoy)
        self.sub_odo_data = rospy.Subscriber("~odo_data", Twist2DStamped, self.cbCtl, queue_size=1)
        self.sub_pid_data = rospy.Subscriber("~pid_data", Twist2DStamped, self.cbCtl, queue_size=1)
        self.sub_move_data = rospy.Subscriber("~move_data", String, self.cbMove, queue_size=1)
        self.sub_mode = rospy.Subscriber("~switch", BoolStamped, self.cbState)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.01)

        # action services
        self.srv_shoot_left = rospy.Service("~shoot_left", Empty, self.cbSrvShootLeft)
        self.srv_shoot_right = rospy.Service("~shoot_right", Empty, self.cbSrvShootRight)
        self.srv_shoot_left = rospy.Service("~shoot_left_2", Empty, self.cbSrvShootLeft_2)
        self.srv_shoot_right = rospy.Service("~shoot_right_2", Empty, self.cbSrvShootRight_2)
        self.srv_shoot_left = rospy.Service("~shoot_left_3", Empty, self.cbSrvShootLeft_3)
        self.srv_shoot_right = rospy.Service("~shoot_right_3", Empty, self.cbSrvShootRight_3)
        self.srv_open_left = rospy.Service("~open_left", Empty, self.cbSrvOpenLeft)
        self.srv_open_right = rospy.Service("~open_right", Empty, self.cbSrvOpenRight)
        self.srv_close_left = rospy.Service("~close_left", Empty, self.cbSrvCloseLeft)
        self.srv_close_right = rospy.Service("~close_right", Empty, self.cbSrvCloseRight)
        self.srv_fetch = rospy.Service("~fetch", Empty, self.cbSrvFetch)
        self.srv_release = rospy.Service("~release", Empty, self.cbSrvRelease)
        self.srv_vertical = rospy.Service("~vertical", Empty, self.cbSrvVertical)
        self.srv_horizontal = rospy.Service("~horizontal", Empty, self.cbSrvHorizontal)
        # Create a timer that calls the process function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.frequency),self.cbControl)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, msg):
        self.joy_msg = msg

    def cbCtl(self, msg):
        self.odo_msg = msg

    def cbControl(self, event):
        if self.joystick_state == True:
            # joystick control
            try:
                self.cbJoyData(self.joy_msg)
            except:
                return
        else:
            # odometry control
            try:
                self.cbCtlData(self.odo_msg)
            except:
                return

    def cbState(self, msg):
        self.joystick_state = msg.data

    def cbSrvFetch(self, req):
        fetch_msg = String()
        fetch_msg.data = "fetch"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvRelease(self, req):
        release_msg = String()
        release_msg.data = "release"
        self.cbMove(release_msg)
        return EmptyResponse()

    def cbSrvVertical(self, req):
        return EmptyResponse()

    def cbSrvHorizontal(self, req):
        return EmptyResponse()
    
    def cbSrvOpenLeft(self, req):
        release_msg = String()
        release_msg.data = "open_left"
        self.cbMove(release_msg)
        return EmptyResponse()

    def cbSrvOpenRight(self, req):
        release_msg = String()
        release_msg.data = "open_right"
        self.cbMove(release_msg)
        return EmptyResponse()

    def cbSrvCloseLeft(self, req):
        fetch_msg = String()
        fetch_msg.data = "close_left"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvCloseRight(self, req):
        fetch_msg = String()
        fetch_msg.data = "close_right"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootLeft(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_left"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootRight(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_right"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootLeft_2(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_left_2"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootRight_2(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_right_2"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootLeft_3(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_left_3"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbSrvShootRight_3(self, req):
        fetch_msg = String()
        fetch_msg.data = "shoot_right_3"
        self.cbMove(fetch_msg)
        return EmptyResponse()

    def cbMove(self, msg):
        # send move according to message
        self.shoot_left_flag = self.shoot_right_flag = 0.0
        self.button_0 = self.button_1 = 2.0
        move = msg.data
        if move == "sleep":
            button_1 = 0.0
        else:
            if move == "fetch":
                self.button_0 = button_0 = 0.0
                self.button_1 = button_1 = 0.0
            elif move == "release":
                self.button_0 = button_0 = 1.0
                self.button_1 = button_1 = 1.0
            elif move == "open_left":
                self.button_0 = button_0 = 1.0
            elif move == "close_left":
                self.button_0 = button_0 = 0.0
            elif move == "open_right":
                self.button_1 = button_1 = 1.0
            elif move == "close_right":
                self.button_1 = button_1 = 0.0
            elif move == "shoot_left":
                self.shoot_left_flag = 1.0
            elif move == "shoot_right":
                self.shoot_left_flag = 2.0
            elif move == "shoot_left_2":
                self.shoot_left_flag = 2.0
            elif move == "shoot_right_2":
                self.shoot_right_flag = 2.0
            elif move == "shoot_left_3":
                self.shoot_left_flag = 3.0
            elif move == "shoot_right_3":
                self.shoot_right_flag = 3.0
            else:
                return
        data_list = translateCTLtoMove(self.shoot_left_flag, # shoot left flag
                                      0.6,   # release position
                                      0.0,   # fetch speed
                                      0.0,   # release speed
                                      self.button_0,
                                      self.button_1)
        
        # write data to serial port
        self.writeData(data_list)
        return
        
        # Write data to serial port
        for t in range(5):
            rospy.loginfo("*************************************************")
            for i, datum in enumerate(data_list):
                self.encoder.write(datum)
                rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))
            self.encoder.write(13)
            self.encoder.write(10)
            rospy.sleep(0.05)

    def cbCtlData(self, msg, plot_everything=False):
        # check fsm state:
        if self.joystick_state == True:
            return
            
        self.shoot_left_flag = self.shoot_right_flag = 0.0
        self.button_0 = self.button_1 = 2.0
        # check protocol type
        if self.protocol_auto_flag:
            # Using protocol for auto-control
            # Translate data from Twisted2D to JoyRemote
            #data_list = translateCTLtoAuto(msg.v_x/2.0,
            #                            msg.v_y/2.0,
            #                            msg.omega/2.0)
            data_list = translateAuto(
                msg.v_x/4.0,
                msg.v_y/4.0,
                msg.omega/4.0,
                0.0, # shoot
                0.6,   # release position
                self.fetch_state,   # fetch state
                0.0,   # release speed
                self.button_0, # fetch!
                self.button_1  # shoot!
            )

            # Check protocol
            if len(data_list) != len(self.protocol):
                print "length data list: ", len(data_list)
                raise Exception("Serial Protocol not matched!")
        else:
            # Translate data from Twisted2D to JoyRemote
            data_list = translateCTLtoRemote(msg.v_x/2.0,
                                        msg.v_y/2.0,
                                        msg.omega/2.0,
                                        0.0,   # Temp set phi, button1, button2 to zero
                                        self.button_0,
                                        self.button_1)
            
            # Check protocol
            if len(data_list) != len(self.protocol):
                print "length data list: ", len(data_list)
                raise Exception("Serial Protocol not matched!")

        # Display
        if plot_everything:
            self.display(data_list)
        
        # write data to serial port
        self.writeData(data_list)
        return
        
        # Write data to serial port
        #rospy.loginfo("*************************************************")
        for i, datum in enumerate(data_list):
            self.encoder.write(datum)
            rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))
        self.encoder.write(13)
        self.encoder.write(10)

    def cbJoyData(self,msg, plot_everything=False):
        # Check fsm state: joystick control
        if self.joystick_state != True:
            return
        # check protocol type
        if self.protocol_auto_flag:
            # Using protocol for auto-control
            # translate data
            data_list = translateAuto(msg.channel_0/5.0,
                                -msg.channel_1/5.0,
                                msg.channel_2/5.0,
                                0.0,
                                msg.channel_4,
                                msg.channel_5,
                                msg.channel_6,
                                msg.channel_7,
                                msg.channel_8)
            
            # Check protocol
            if len(data_list) != len(self.protocol_auto):
                print "length data list: ", len(data_list)
                print "length protocol: ", len(self.protocol_auto)
                raise Exception("Serial Protocol not matched!")
        else:
            # Using protocol for remote-control
            # translate data
            data_list = translateRemote(msg.channel_0/2.0,
                                -msg.channel_1/2.0,
                                msg.channel_2/3.0,
                                msg.channel_3/3.0,
                                msg.button_0,
                                msg.button_1)
            
            # Check protocol
            if len(data_list) != len(self.protocol_remote):
                print "length data list: ", len(data_list)
                print "length protocol: ", len(self.protocol_remote)
                raise Exception("Serial Protocol not matched!")

        # Display
        if plot_everything:
            self.display(data_list)

        # write data to serial port
        self.writeData(data_list)
        return

        # Write data to serial port old version
        rospy.loginfo("*************************************************")        
        # Then send joystick data
        for i, datum in enumerate(data_list):
            self.encoder.write(datum)
            if self.display_buffer == self.display_every - 1:
                rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))
        self.encoder.write(13)
        self.encoder.write(10)
        self.display_buffer = (self.display_buffer + 1) % self.display_every
        # To meet the protocal send 12 zeros
        # for i in range(12):
        #     self.encoder.write(0)

        self.Rate.sleep()

    def writeData(self, data_list):
        try:
            # Write data to serial port
            rospy.loginfo("*************************************************")        
            # Then send joystick data
            for i, datum in enumerate(data_list):
                self.encoder.write(datum)
                rospy.loginfo("[%s] signal: %s: %s" %(self.node_name, self.protocol[i], str(datum)))
            self.encoder.write(13)
            self.encoder.write(10)
        except Exception as e:
            print e
            reconnect_success = False
            while(reconnect_success==False):
                # try to reconnect
                try:
                    self.encoder = Encoder(self.baudrate)
                    reconnect_success = True
                except Exception as e:
                    print e
                    rospy.sleep(0.1)

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
