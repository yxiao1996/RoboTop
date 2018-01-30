#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import CCD_pose, Twist2DStamped

class ccd_control_node(object):
    def __init__(self):
        # Setup buffers
        self.d = 0.0
        self.phi = 0.0
        self.v = 0.0
        self.omega = 0.0

        # Setup parameters
        self.d_offset = rospy.get_param("~d_offset")
        self.K_d = rospy.get_param("~K_d")
        self.K_phi = rospy.get_param("~K_phi")
        self.v_bar = rospy.get_param("~v_bar")

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped, queue_size=1)
        # Setup subscriber
        self.sub_pose = rospy.Subscriber("/Robo/ccd_filter_node/pose", CCD_pose, self.cdPose)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cdPub function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cdPub)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cdPose(self,msg):
        self.d = msg.d
        self.phi = msg.phi
        rospy.loginfo("[%s] Pose: (d: %s, phi: %s)" %(self.node_name,msg.d, msg.phi))
        #self.cdPub()

    def cdPub(self, event):
        corss_track_error = self.d - self.d_offset
        heading_error = self.phi
        omega = self.K_d * corss_track_error + self.K_phi * heading_error
        v = self.v_bar
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_control_node', anonymous=False)

    # Create the NodeName object
    node = ccd_control_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()