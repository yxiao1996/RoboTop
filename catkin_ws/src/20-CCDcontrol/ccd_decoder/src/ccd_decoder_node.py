#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from ccd_decoder.ccd import CCD as Decoder
from Tkinter import *
from robocon_msgs.msg import CCD_msg

class ccd_decoder_node(object):
    def __init__(self):
        # Init Decoder
        #root = Tk()
        #root.title("CCD Helper")
        self.decoder = Decoder()
        
        # setup frequency
        self.frequency = 500.0

        # setup timer
        self.start_time = rospy.Time.now()

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_ccd_msg = rospy.Publisher("~ccd_msg",CCD_msg, queue_size=1)
        # Setup subscriber
        #self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.01)
        # Create a timer that calls the process function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.frequency),self.cbprocess)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    #def cbTopic(self,msg):
        #rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

    def cbprocess(self,event):
        rospy.loginfo("[%s]" %(rospy.Time.now()-self.start_time))
        self.start_time = rospy.Time.now()
        ccd_msg = CCD_msg()        
        ccd_data = self.decoder.read()
        # print ccd_data
        ccd_msg.data = ccd_data
        self.pub_ccd_msg.publish(ccd_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_decoder_node', anonymous=False)

    # Create the NodeName object
    node = ccd_decoder_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()