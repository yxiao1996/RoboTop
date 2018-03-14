#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import threading

class ccd_detector_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.bridge = CvBridge()
        self.state = "init"
        self.active = True
        self.plot_detect = True
        self.compressed = True

        # Setup publishers
        self.pub_centroid = rospy.Publisher("/line_centroid", Float64, queue_size=1)
        # Setup subscriber
        if not self.compressed:
            # use raw image
            self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImage, queue_size=1)
        else:
            self.sub_image = rospy.Subscriber("~image_compressed", CompressedImage, self.cbImage, queue_size=1)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.1)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.processImg)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImage(self,image_msg):
        if not self.active:
            return
        #self.rgb = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.state = "default"
        self.processImg(image_msg)
        # Start a daemon thread to process the image
        #thread = threading.Thread(target=self.processImg,args=(image_msg,))
        #thread.setDaemon(True)
        #thread.start()
        # Returns rightaway

    def processImg(self, image_msg):
        if not self.active:
            return
        if self.state == "init":
            return
        if self.compressed:
            # decode image from compressed
            try:
                np_arr = np.fromstring(image_msg.data, np.uint8)
                self.rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except ValueError as e:
                raise Exception("can not decode image")
                return
        else:
            self.rgb = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")

        #plt.imshow(self.rgb)
        #plt.show()

        # Crop image
        height = 480
        width = 640 
        decenter = 100
        row_to_watch = 320

        rgb = self.rgb[height/2+decenter : height][1:width]

        # Transfer RBG image into HSV image
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    
        # Calculate mask 
        sensitivity = 200
        lower_white = np.array([0,0,255-sensitivity])
        upper_white = np.array([255,sensitivity,255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Apply mask to cropped image
        bi_img = cv2.bitwise_and(rgb, rgb, mask=mask)

        # Calculate centroid
        M = cv2.moments(mask)
        if M['m00']!=0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX = 319
            cY = 100
        cv2.circle(bi_img, (cX, cY), 7, (255, 0, 0), -1)
        
        if self.plot_detect:
            cv2.imshow("centroid", bi_img)
            cv2.waitKey(1)
        
        line_centroid_msg = Float64()
        line_centroid_msg.data = cX
        self.pub_centroid.publish(line_centroid_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_detector_node', anonymous=False)

    # Create the NodeName object
    node = ccd_detector_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
