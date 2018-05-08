#!/usr/bin/python
import cv2
import rospy
import numpy as np
from circle_detector.circle_detector import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from robocon_msgs.msg import BoolStamped
from sklearn import mixture

class CircleDetectorNode():
    def __init__(self):
        self.node_name = "circle_detector"
        self.bridge = CvBridge()
        self.framerate = 50
        self.detector = CircleDetector()
        self.status = "init"
        self.debug = False
        self.data_size = 50
        self.trigger = False
        self.fit_circles = None
        # Publishers
        self.pub_roi = rospy.Publisher("~roi", Image, queue_size=20)
        self.pub_set_ref = rospy.Publisher("~set_ref", BoolStamped, queue_size=1)
        self.pub_debug = rospy.Publisher("~image_with_circles", Image, queue_size=1)
        self.pub_bw_img = rospy.Publisher("~bw_image", Image, queue_size=1)
        # Subscribers
        self.sub_trigger = rospy.Subscriber("~trigger", BoolStamped, self.cbTrigger, queue_size=1)
        self.sub_image = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.cbImage, queue_size=20)
        # Timers
        rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate), self.mainLoop)
    
    def cbTrigger(self, msg):
        self.trigger = msg.data
        if msg.data == True:
            self.data = []
            rospy.loginfo("[%s] get trigger, start capturing image" %(self.node_name))

    def cbImage(self, image_msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        self.rbg = cv_image
        self.status = "default"
        if self.fit_circles is not None:
            # Crop image according to data
            circle = self.fit_circles[0]
            x = int(circle[0])
            y = int(circle[1])
            r = int(circle[2])
            x1 = x - r
            if x1 < 0:
                x1 = 0
            x2 = x + r
            if x2 < 0:
                x2 = 0
            y1 = y - r
            if y1 < 0:
                y1 = 0
            y2 = y + r
            if y2 < 0:
                y2 = 0
            # Crop the square area
            roi = self.rbg[y1:y2, x1:x2, :]
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            roi = cv2.resize(roi, (100, 100))
            #msg = CompressedImage()
            #msg.header.stamp = image_msg.header.stamp
            #msg.format = "yuyv"
            #msg.data = np.array(cv2.imencode('.yuyv', roi)[1]).tostring()
            msg = self.bridge.cv2_to_imgmsg(roi, 'rgb8')
            msg.header.stamp = image_msg.header.stamp
            self.pub_roi.publish(msg)

    def fitData(self):
        self.trigger = False
        rospy.loginfo("[%s] fitting data" %(self.node_name))
        # fit the data using gaussian mixture
        clf = mixture.GaussianMixture(n_components=2, covariance_type='full')
        clf.fit(self.data)
        # Publish set ref message to filter
        set_msg = BoolStamped()
        set_msg.header.stamp = rospy.Time.now()
        self.pub_set_ref.publish(set_msg)
        return clf.means_

    def mainLoop(self, _event):
        if not self.trigger:
            return
        if self.status == "default":
            # Detect circles in image
            self.detector.setImage(self.rbg)
            circles, bw_image = self.detector.detectCircles('green')

            # Crop Image according to circles area
            if len(circles) > 0:
                circle = circles[0, 0]
                if len(self.data) < self.data_size:
                    rospy.loginfo("[%s] capturing image %s / %s" %(self.node_name, len(self.data), self.data_size))
                    self.data.append(circle)
                    #circle = circles[0, 1]
                    #self.data.append(circle)
                else:
                    self.fit_circles = self.fitData()
                    self.detector.drawCircles_(self.fit_circles)
                    img1 = self.detector.getImage()
                    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                    image_msg = self.bridge.cv2_to_imgmsg(img1, 'rgb8')
                    self.pub_debug.publish(image_msg)
                pub = False
                if pub:
                    x = circle[0]
                    y = circle[1]
                    r = circle[2]
                    x1 = x - r
                    if x1 < 0:
                        x1 = 0
                    x2 = x + r
                    if x2 < 0:
                        x2 = 0
                    y1 = y - r
                    if y1 < 0:
                        y1 = 0
                    y2 = y + r
                    if y2 < 0:
                        y2 = 0
                    # Crop the square area
                    #print np.array(self.rbg).shape
                    #print (x1, x2, y1, y2)
                    roi = self.rbg[y1:y2, x1:x2, :]
                    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
                    roi = cv2.resize(roi, (100, 100))
                    image_msg = self.bridge.cv2_to_imgmsg(roi, 'rgb8')
                    self.pub_roi.publish(image_msg)
            
            if self.debug:
                if len(circles) > 0:
                    print len(circles[0])
                self.detector.drawCircles(circles)
                img1 = self.detector.getImage()
                img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
                image_msg = self.bridge.cv2_to_imgmsg(img1, 'rgb8')
                self.pub_debug.publish(image_msg)
                bw_image = cv2.cvtColor(bw_image, cv2.COLOR_GRAY2RGB)
                image_msg = self.bridge.cv2_to_imgmsg(bw_image, 'rgb8')
                self.pub_bw_img.publish(image_msg)
        
if __name__ == '__main__':
    rospy.init_node('Circle_detector_node',anonymous=False)
    vo = CircleDetectorNode()
    rospy.spin()