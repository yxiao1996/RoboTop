#!/usr/bin/python
from ball_detector.ball_detector import *
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import cv2

class ballDetectorNode():
    def __init__(self):
        self.node_name = "ball_detector"
        self.bridge = CvBridge()
        self.framerate = 20
        self.detector = BallDetector()
        self.status = "init"
        # Publishers
        self.pub_debug = rospy.Publisher("~debug", Image, queue_size=1)
        # Subscribers
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.cbImage, queue_size=1)
        # Timers
        rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate), self.mainLoop)
    
    def cbImage(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.rbg = cv_image
        self.status = "default"

    def mainLoop(self, _event):
        if self.status == "default":
            self.detector.setImage(self.rbg)
            balls, debug = self.detector.detectBall()
            self.detector.drawCircles(balls)
            
            img1 = self.detector.getImage()
            img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
            debug = cv2.cvtColor(debug, cv2.COLOR_GRAY2RGB)
            image_msg = self.bridge.cv2_to_imgmsg(debug, 'rgb8')
            self.pub_debug.publish(image_msg)
            #cv2.imshow('image', img1)
            #cv2.waitKey(2)
        
if __name__ == '__main__':
    rospy.init_node('ball_detector_node',anonymous=False)
    vo = ballDetectorNode()
    rospy.spin()