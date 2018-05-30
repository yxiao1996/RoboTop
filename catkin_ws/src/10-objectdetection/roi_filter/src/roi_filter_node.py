#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from sklearn.linear_model import LogisticRegression as LR
from sklearn import svm
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty

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
        self.tmp_data = None

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        # Setup subscriber
        self.sub_roi = rospy.Subscriber("~roi", Image, self.cbRoi, queue_size=20)
        self.sub_set_ref = rospy.Subscriber("~set_ref", BoolStamped, self.cbSetRef, queue_size=1)
        # Setup Service
        self.srv_add_pos = rospy.Service("~add_positive", Empty, self.cbSrvAddPos)
        self.srv_add_neg = rospy.Service("~add_negative", Empty, self.cbSrvAddNeg)
        self.srv_fit = rospy.Service("~fit", Empty, self.cbSrvFit)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSrvAddPos(self, req):
        if self.tmp_data is None:
            return
        # get data param
        training = rospy.get_param("/training")
        # append temp data to training data
        training[0]["x"].append(self.tmp_data)
        # append 1 to training label
        training[1]["y"].append(1)
        # set
        rospy.set_param("/training", training)
        rospy.loginfo("append positive data")
        return EmptyResponse()

    def cbSrvAddNeg(self, req):
        if self.tmp_data is None:
            return
        # get data param
        training = rospy.get_param("/training")
        # append temp data to training data
        training[0]["x"].append(self.tmp_data)
        # append 0 to training label
        training[1]["y"].append(0)
        # set 
        rospy.set_param("/training", training)
        rospy.loginfo("append negative data")
        return EmptyResponse()

    def cbSrvFit(self, req):
        # fetch data 
        training = rospy.get_param("/training")
        raw_x = training[0]["x"]
        y = training[1]["y"]
        x = []
        for raw in raw_x:
            x.append(self.preprocess(raw))
        print "training data", x
        print "training label", y

        use_lr = False
        use_svm = True
        if use_lr:
            # fit logistic regression
            lr = LR()
            lr.fit(x, y)
        elif use_svm:
            clf = svm.SVC(kernel='rbf', gamma='auto')
            clf.fit(x, y)
        else:
            return

        # Plot the decision boundary. For that, we will assign a color to each
        # point in the mesh [x_min, x_max]x[y_min, y_max].
        h = .02  # step size in the mesh
        x_list = []
        y_list = []
        for p in x:
            x_list.append(p[0])
            y_list.append(p[1])
        x_min, x_max = np.min(x_list) - 0.5, np.max(x_list) + 0.5
        y_min, y_max = np.min(y_list) - 0.5, np.max(y_list) + 0.5
        xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))
        if use_lr:
            Z = lr.predict(np.c_[xx.ravel(), yy.ravel()])
        elif use_svm:
            Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
        else:
            return
        # Put the result into a color plot
        Z = Z.reshape(xx.shape)
        plt.figure(1, figsize=(4, 3))
        plt.pcolormesh(xx, yy, Z)

        # Plot also the training points
        #print x_list
        plt.scatter(x_list, y_list, c=y, edgecolors='k')
        plt.xlabel('signal width')
        plt.ylabel('peak value')

        plt.xlim(xx.min(), xx.max())
        plt.ylim(yy.min(), yy.max())
        plt.xticks(())
        plt.yticks(())

        plt.show()
        return EmptyResponse()

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
                if delta > 0.4:
                    out_data[i] = delta
                else:
                    out_data[i] = 0.0
            print out_data
            print len(self.data)
            self.tmp_data = out_data

            self.ref = None

    def preprocess(self, data):
        # find how many non-negative element in data list
        count = 0
        # find peak value in data list
        peak = 0
        for datum in data:
            if datum != 0:
                count += 1
            if datum > peak:
                peak = datum
        return [count, peak]

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