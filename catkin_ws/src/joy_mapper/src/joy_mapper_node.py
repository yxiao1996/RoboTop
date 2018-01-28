#!/usr/bin/env python
import rospy
#from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyMapperNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist, queue_size=1)
        # Setup subscriber
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.cbJoy)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.1)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)
        
        self.v_gain = 100
        self.omega_gain = 10

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self,joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)
        #rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

    def cbTimer(self,event):
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("duckietown")
        self.pub_car_cmd.publish(msg)
    
    def publishControl(self):
        car_cmd_msg = Twist()
        car_cmd_msg.angular.z = self.omega_gain * self.joy.axes[0]
        car_cmd_msg.linear.x = self.v_gain * self.joy.axes[1]
        self.pub_car_cmd.publish(car_cmd_msg)

    def processButtons(self, joy_msg):
        
        return

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('joy_mapper_node', anonymous=False)

    # Create the NodeName object
    node = JoyMapperNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()