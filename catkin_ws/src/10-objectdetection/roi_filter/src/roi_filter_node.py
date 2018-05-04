#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError

class roi_filter_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.capture_time = 3.0
        self.ref = None
        self.set_ref = False
        self.data = []
        self.bridge = CvBridge()

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_roi = rospy.Subscriber("~roi", Image, self.cbRoi, queue_size=20)
        self.sub_set_ref = rospy.Subscriber("~set_ref", BoolStamped, self.cbSetRef, queue_size=1)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSetRef(self, msg):
        self.set_ref =True
        self.data = []

    def cbRoi(self,msg):
        if self.set_ref:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.ref = img
            self.height = np.array(img).shape[0]
            self.width = np.array(img).shape[1]
            self.start_time = msg.header.stamp
            self.set_ref = False
            return
        if self.ref is None:
            return
        # calcualte pixel difference
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        stamp = msg.header.stamp
        delta_t = (stamp - self.start_time).to_sec()
        print delta_t
        if delta_t < self.capture_time:
            print "capturing image..."
            self.data.append(img)
        else:
            out_data = []
            min_difference = 100.0
            for image in self.data:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                difference = 0.0
                for h in range(self.height):
                    for w in range(self.width):
                        delta = abs(int(image[h, w]) - int(self.ref[h, w]))
                        #print delta
                        difference += delta
                difference = difference / (self.width * self.height)
                out_data.append(difference)
                if difference < min_difference:
                    min_difference = difference
            for i, datum in enumerate(out_data):
                delta = datum - min_difference
                if delta > 0.3:
                    out_data[i] = delta
                else:
                    out_data[i] = 0.0
            print out_data
            print len(self.data)

            self.ref = None

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('roi_filter_node', anonymous=False)

    # Create the NodeName object
    node = roi_filter_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()