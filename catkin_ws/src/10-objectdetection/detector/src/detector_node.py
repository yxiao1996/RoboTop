#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import Image
from robocon_msgs.msg import BoolStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2

class detector_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        
        self.active = False
        self.state = "init"
        self.bridge = CvBridge()
        self.detector = cv2.CascadeClassifier('cascade.xml')
        self.detector.load('/home/yxiao1996/workspace/RoboVision/modules/object_detector/gen/cascade.xml')
        # Setup publishers
        #self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.cbImage, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.05)
        # Create a timer that calls the processImg function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.processImg)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImage(self,msg):
        if not self.active:
            return
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #self.rgb[:, :, [0, 2]] = self.rgb[:, :, [2, 0]]
        #rgb[:, :, [0, 2]] = rgb[:, :, [2, 0]]
        self.state = "default"


    def cbSwitch(self, msg):
        next_state = msg.data
        self.active = next_state

    def processImg(self,event):
        if not self.active:
            return
        if self.state == "init":
            return
        gray = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2GRAY)
            
        targets = self.detector.detectMultiScale(gray, 1.3, 6)
        img = self.rgb.copy()
        
        for (x, y, w, h) in targets:
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)
            roi_gray = gray[x:x+w, y:y+h]
            roi_color = img[x:x+w, y:y+h]
            break
        cv2.imshow('detector test', img)
        cv2.waitKey(1)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('detector_node', anonymous=False)

    # Create the NodeName object
    node = detector_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()