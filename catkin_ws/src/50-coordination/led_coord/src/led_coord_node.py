#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import Image
from robocon_msgs.msg import BoolStamped
from cv_bridge import CvBridge, CvBridgeError


class Talker(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.active = False
        self.bridge = CvBridge()
        
        # HSV threshold
        self.redLower = np.array([100, 100, 100])
        self.redUpper = np.array([125, 255, 255])

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        self.sub_image = rospy.Subscriber("~image", Image, self.cbImage)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self,msg):
        self.active = msg.data

    def cbImage(self, msg):
        if not self.active:
            return
        
        self.bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Detect static led
        hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.redLower, self.redUpper)
        # 腐蚀操作
        mask = cv2.erode(mask, None, iterations=2)
        # 膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
        mask = cv2.dilate(mask, None, iterations=2)
        # 轮廓检测
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        print cnts

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = Talker()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()