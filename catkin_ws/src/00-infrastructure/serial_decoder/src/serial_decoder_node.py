#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String #Imports msg
#from serial_decoder.ccd import CCD as Decoder
from serial_decoder.decoder import Decoder
from serial_decoder.arduino_decoder import ArduinoDecoder
from Tkinter import *
from robocon_msgs.msg import CCD_data, Pose2DStamped, BoolStamped
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import plot
import struct
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from std_msgs.msg import Int32

class decoder_node(object):
    def __init__(self):
        self.debug_flag = True
        self.arduino_flag = True

        # Init Decoder
        self.baudrate = rospy.get_param("~baudrate")
        
        connect_success = False
        while(connect_success==False):
            # try to connect
            try:
                self.decoder = Decoder(self.baudrate)
                self.arduino_decoder = ArduinoDecoder(19200)
                connect_success = True
            except Exception as e:
                print e
                rospy.sleep(0.1)
        
        # Set decoder protocol
        self.total_length = rospy.get_param("~total_length")
        self.data_length = rospy.get_param("~data_length")
        self.data_order = rospy.get_param("~data_order")
        self.verify_switch = rospy.get_param("~verify_switch")
        self.ccd_switch = rospy.get_param("~ccd_switch")
        self.odo_switch = rospy.get_param("~odo_switch")

        self.ack_codebook = {
            "left_shoot": 1,
            "right_shoot": 1,
            "left_open": 1,
            "right_open": 1,
            "left_close": 0,
            "right_close": 0
        }

        # offset buffer
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.theta_offset = 0.0
        self.x_buf = 0.0
        self.y_buf = 0.0
        self.t_buf = 0.0

        # setup frequency
        self.frequency = 5.0
        self.Rate = rospy.Rate(self.frequency)
        self.plot_debug = True
        self.display_init = False

        # setup timer
        self.start_time = rospy.Time.now()

        self.select = 0
        self.ccd_msg = CCD_data()
        self.odo_buffer = [0.0, 0.0, 0.0]

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        rospy.loginfo("[%s]: baudrate: [%s]" %(self.node_name, self.baudrate))

        # Setup publishers
        self.pub_odo_msg = rospy.Publisher("~odo_msg", Pose2DStamped, queue_size=1)
        self.pub_odo_debug = rospy.Publisher("~odo_debug", PoseStamped, queue_size=1)
        self.pub_ccd_msg = rospy.Publisher("~ccd_msg",CCD_data, queue_size=1)
        self.pub_debug = rospy.Publisher("~debug", Joy, queue_size=1)
        self.pub_ack_left_shoot = rospy.Publisher("~ack_left_shoot", BoolStamped, queue_size=1)
        self.pub_ack_right_shoot = rospy.Publisher("~ack_right_shoot", BoolStamped, queue_size=1)
        self.pub_ack_left_open = rospy.Publisher("~ack_left_open", BoolStamped, queue_size=1)
        self.pub_ack_right_open = rospy.Publisher("~ack_right_open", BoolStamped, queue_size=1)
        self.pub_delta_y = rospy.Publisher("~delta_y", Int32, queue_size=1)
        
        # Setup Service
        self.srv_offset = rospy.Service("~set_offset", Empty, self.cbSrvOffset)
        self.lit = rospy.Service("~lit", Empty, self.cbLit)
        self.die = rospy.Service("~die", Empty, self.cbDie)

        # Setup serviceproxy
        self.macro0 = rospy.ServiceProxy('/Robo/task_planner_node/macro0', Empty)
        self.macro1 = rospy.ServiceProxy('/Robo/task_planner_node/macro1', Empty)
        self.macro2 = rospy.ServiceProxy('/Robo/task_planner_node/macro2', Empty)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.01)
        # Create a timer that calls the process function every 1.0 second
        self.debug_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbdebug)

        #rospy.sleep(rospy.Duration.from_sec(1))
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSrvOffset(self,req):
        # update robot state parameter
        robot_state = rospy.get_param("/robot_state")
        robot_state[0]["offset"][0] = robot_state[0]["odom"][0]
        robot_state[0]["offset"][1] = robot_state[0]["odom"][1]
        robot_state[0]["offset"][2] = robot_state[0]["odom"][2]
        rospy.set_param("/robot_state", robot_state)
        return EmptyResponse()

    def cbLit(self, req):
        self.writeArduino(1)
        return EmptyResponse()

    def cbDie(self, req):
        self.writeArduino(0)
        return EmptyResponse()

    def cbdebug(self, event):
        try:
            # Read data list from serial port
            data_list = self.decoder.read_debug(self.total_length)

            # Parse and process data
            self.parse_data(data_list)
        except Exception as e:
            rospy.loginfo(e)
            reconnect_success = False
            while(reconnect_success==False):
                # try to reconnect
                try:
                    self.decoder = Decoder(self.baudrate)
                    reconnect_success = True
                except Exception as e:
                    print e
                    rospy.sleep(0.1)

        # read data from arduino
        if self.arduino_flag:
            try:
                # read distance sensor data from arduino
                ard_data_list = self.arduino_decoder.read_debug()
                print ard_data_list
                
                # publish data to odometry controllor
                if ard_data_list is not None:
                    self.publish_arduino(ard_data_list)
            except Exception as e:
                rospy.loginfo(e)
                reconnect_success = False
                while(reconnect_success==False):
                    # try to reconnect
                    try:
                        self.arduino_decoder = ArduinoDecoder(19200)
                        reconnect_success = True
                    except Exception as e:
                        print e
                        rospy.sleep(0.1)

    def writeArduino(self, led=0, ack1=1, ack2=1):
        data = 0
        if led == 0:
            data = "L"
        else:
            data = "X"
        if ack1 == 1:
            data = 48
        if ack2 == 1:
            data = 49
        self.arduino_decoder.write(data)
        #data_list = [10, led, ack1, ack2]
        #for datum in data_list:
        #    self.arduino_decoder.write(datum)
        rospy.loginfo("write data to arduino [%s]" %(data))


    def publish_arduino(self, data_list):
        bt_1 = data_list[4] 
        bt_2 = data_list[5]
        if bt_1 == 1 or bt_2 == 1:
            self.x_offset = self.x_buf
            self.y_offset = self.y_buf
            self.theta_offset = self.t_buf
            if bt_1 == 1:
                self.macro1()
                self.writeArduino(0, 1, 0)
            else:
                self.macro2()
                self.writeArduino(0, 0, 1)

        # four elements in data list
        y_1 = data_list[0] # left hand side sensor
        y_2 = data_list[1] # right hand side sensor
        x_1 = data_list[2]
        delta_y = y_1 - y_2
        y_msg = Int32()
        y_msg.data = delta_y
        self.pub_delta_y.publish(y_msg)

    def cbprocess(self,event):
        
        self.start_time = rospy.Time.now()        
        ccd_num, ccd_mid = self.decoder.read()
        if ccd_num == 0:
            self.ccd_msg.ccd_0 = np.uint8(ccd_mid)
        elif ccd_num == 1:
            self.ccd_msg.ccd_1 = np.uint8(ccd_mid)
        elif ccd_num == 2:
            self.ccd_msg.ccd_2 = np.uint8(ccd_mid)
        else:
            self.ccd_msg.ccd_3 = np.uint8(ccd_mid)
        rospy.loginfo("[%s] ccd: [%s], mid:[%s]" %(self.node_name, ccd_num, ccd_mid))
        # print ccd_data
        #ccd_msg.data = ccd_data
        if self.select % 4 == 3:
            self.pub_ccd_msg.publish(self.ccd_msg)
        else:
            self.select += 1

    def display(self, data_list):
        # plot everything using bars
        if self.display_init == False:
            self.fig, self.axes = plt.subplots(6, 1)
            self.display_init = True
        bars = []
        for datum in data_list:
            bar = plot.gen_bar(datum)
            bars.append(bar)
        plot.plot_bar(bars, self.axes, display_encoder=False)
        plt.pause(0.000001)

    def parse_data(self, data_list_raw):
        data_list = data_list_raw[2]
        segment_1 = data_list[0 : self.data_length[0]]
        segment_2 = data_list[self.data_length[0] : self.data_length[0]+self.data_length[1]]
        segment_3 = data_list[self.data_length[0]+self.data_length[1] : self.total_length]
        data_segments = [segment_1, segment_2, segment_3]

        # Parse data list
        for i, segment in enumerate(data_segments):
            if self.data_order[i] == "verify":
                self.verify(segment)
            if self.data_order[i] == "ccd":
                self.ccd_process(segment)
            if self.data_order[i] == "odo":
                self.odo_process(segment)
        
        # self.Rate.sleep()
    
    def verify(self, data_list, plot_everything=False):
        if not self.verify_switch:
            return

        #if len(data_list) > 0:
            #print data_list
        # Display
        if plot_everything:
            self.display(data_list)

        try: 
            # Check protocal
            assert len(data_list) == self.data_length[0]
            data = []
            left_open = False
            left_shoot = False
            right_open = False
            right_shoot = False
            for raw in data_list:
                datum = struct.unpack('B', "".join(raw))[0]
                data.append(datum)
                rospy.loginfo("revice ack info [%s]"%(datum))
            if data[0] == 23 and data[1] == 33:
                # Setup offset
                # update robot state parameter
                robot_state = rospy.get_param("/robot_state")
                robot_state[1]["offset"][0] = robot_state[0]["odom"][0]
                robot_state[1]["offset"][1] = robot_state[0]["odom"][1]
                robot_state[1]["offset"][2] = robot_state[0]["odom"][2]
                rospy.loginfo("set up offset [%s]"%(robot_state[1]["offset"]))
                rospy.set_param("/robot_state", robot_state)
            elif data[0] == self.ack_codebook["left_shoot"]:
                # ack signal to shoot task
                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = True
                self.pub_ack_left_shoot.publish(msg)
            if data[0] == self.ack_codebook["left_open"]:
                left_open = True
            if data[1] == self.ack_codebook["right_open"]:
                right_open = True
            if data[2] == self.ack_codebook["left_shoot"]:
                left_shoot = True
            if data[3] == self.ack_codebook["right_shoot"]:
                right_shoot = True
            # ack signal to left fetch task
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = left_open
            self.pub_ack_left_open .publish(msg)
            # ack signal to left shoot task
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = left_shoot
            self.pub_ack_left_shoot.publish(msg)
            # ack signal to right fetch task
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = right_open
            self.pub_ack_right_open .publish(msg)
            # ack signal to right shoot task
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = right_shoot
            self.pub_ack_right_shoot.publish(msg)
        except Exception as e:
            rospy.loginfo(e)
        return

        # Debug message
        joy_msg = Joy()        
        debug_0 = float(data_list[0]) / 128.0 - 1.0
        debug_1 = float(data_list[1]) / 128.0 - 1.0
        debug_2 = float(data_list[2]) / 128.0 - 1.0
        debug_3 = float(data_list[3]) / 128.0 - 1.0
        for i in range(8):
            joy_msg.axes.append(0)
        joy_msg.axes[4] = debug_0
        joy_msg.axes[3] = debug_1
        ("**************************************************")
        rospy.loginfo("[%s] ch 0: [%s] ch 1: [%s]" %(self.node_name, debug_0, debug_1))
        rospy.loginfo("[%s] ch 2: [%s] ch 3: [%s]" %(self.node_name, debug_2, debug_3))
        self.pub_debug.publish(joy_msg)

        return

    def ccd_process(self, data):
        if not self.ccd_switch:
            return
        #if len(data) > 0:
            #print data
        # Decode CCD data
        pose_x = 0.0
        pose_y = 0.0
        pose_theta = 0.0

        # Publish pose data
        pose_msg = Pose2DStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.x = pose_x
        pose_msg.y = pose_y
        pose_msg.theta = pose_theta
        self.pub_odo_msg.publish(pose_msg)

    def odo_process(self, data):
        if not self.odo_switch:
            return
        #if len(data) > 0:
            #print data
        #robot_state = rospy.get_param("/robot_state")
        #print robot_state
        # Decode odometry data
        #data_pre = data[0:2]
        #data_post = data[-2:]
        odom_data = data
        
        try:
            # Check header and tail of each data segment
            #if struct.unpack('B', data_pre[0])[0] != 13 or struct.unpack('B', data_pre[1])[0] != 10:
                #print struct.unpack('B', data_pre[0]), struct.unpack('B', data_pre[1])
            #    return
            #if struct.unpack('B', data_post[0])[0] != 10 or struct.unpack('B', data_post[1])[0] != 13:
            #    return

            rawValue = []
            for i in range(len(odom_data) / 4):
                neg_raw = odom_data[i*4:(i+1)*4]
                pos_raw = neg_raw[::-1]
                raw = struct.unpack('f', "".join(neg_raw))[0]
                rawValue.append(raw)
            current_x = -rawValue[3] - self.x_offset
            current_y = rawValue[4] - self.y_offset
            current_theta = rawValue[0] - self.theta_offset
            #print "%10.4f, %10.4f, %10.4f" % (-rawValue[3], rawValue[4], rawValue[0]) 
            rospy.loginfo("%10.4f, %10.4f, %10.4f" % (current_x, current_y, current_theta)) 
            check = True
            if check:
                # Data check
                x = -rawValue[3]
                y = rawValue[4]
                theta = rawValue[0]
                try:
                    delta_x = x - self.odo_buffer[0]
                    self.odo_buffer[0] = x
                    delta_y = y - self.odo_buffer[1]
                    self.odo_buffer[1] = y
                    delta_theta = theta - self.odo_buffer[2]
                    self.odo_buffer[2] = theta
                    if abs(delta_x) > 700 or abs(delta_y) > 700 or abs(delta_theta) > 30:
                        return
                except:
                    return

            debug = False
            if debug:
                # Publish debug message
                odo_debug = PoseStamped()
                odo_debug.header.stamp = rospy.Time.now()
                odo_debug.header.frame_id = '0'
                odo_debug.pose.position.x = rawValue[3]
                odo_debug.pose.position.y = rawValue[4]
                self.pub_odo_debug.publish(odo_debug)

            # Publish message to odometry controller
            odo_msg = Pose2DStamped()
            odo_msg.x = current_x #-rawValue[3]
            odo_msg.y = current_y #rawValue[4]
            odo_msg.theta = current_theta #rawValue[0]
            odo_msg.header.stamp = rospy.Time.now()
            self.pub_odo_msg.publish(odo_msg)

            self.x_buf = current_x
            self.y_buf = current_y
            self.t_buf = current_theta
            # update robot state parameter
            #robot_state = rospy.get_param("/robot_state")
            #robot_state[0]["odom"][0] = -rawValue[3]
            #robot_state[0]["odom"][1] = rawValue[4]
            #robot_state[0]["odom"][2] = rawValue[0]
            #rospy.set_param("/robot_state", robot_state)
            #print robot_state
        except: 
            return

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('decoder_node', anonymous=False)

    # Create the NodeName object
    node = decoder_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
