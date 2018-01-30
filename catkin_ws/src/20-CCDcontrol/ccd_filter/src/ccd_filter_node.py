#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import CCD_data, CCD_pose
import math

class ccd_filter_node(object):
    def __init__(self):
        # Setup buffers
        self.data_0 = 0
        self.data_1 = 0
        self.data_2 = 0
        self.data_3 = 0   

        # Setup parameter
        self.K_d = rospy.get_param("~K_d")
        self.D_phi = rospy.get_param("~D_phi")

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_pose = rospy.Publisher("~pose",CCD_pose, queue_size=1)
        # Setup subscriber
        self.sub_ccd_data = rospy.Subscriber("/Robo/ccd_decoder_node/ccd_msg", CCD_data, self.cdData)
        
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cdData(self,msg):
        self.data_0 = msg.ccd_0
        #rospy.loginfo("[%s]: (%s, %s)" %(self.node_name,0, msg.ccd_0))
        self.data_2 = msg.ccd_1
        #rospy.loginfo("[%s]: (%s, %s)" %(self.node_name,1, msg.ccd_1))
        self.data_3 = msg.ccd_2
        #rospy.loginfo("[%s]: (%s, %s)" %(self.node_name,2, msg.ccd_2))
        self.data_3 = msg.ccd_3
        rospy.loginfo("[%s]: (%s, %s, %s, %s)" %(self.node_name, msg.ccd_0, msg.ccd_1, msg.ccd_2, msg.ccd_3))
        self.process()

    def process(self):
        d = ((self.data_0 - self.data_2)/2 - 64) * self.K_d
        phi = math.atan((self.data_0 - self.data_2) / self.D_phi)
        pose_msg = CCD_pose()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.d = d
        pose_msg.phi = phi
        pose_msg.in_lane = True
        self.pub_pose.publish(pose_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_filter_node', anonymous=False)

    # Create the NodeName object
    node = ccd_filter_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()