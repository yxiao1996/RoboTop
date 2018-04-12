#!/usr/bin/env python
import rospy
from std_msgs.msg import String,  Int8#Imports msg
from robocon_msgs.msg import BoolStamped

class tsm_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        self.active = False

        # Macro task buffer
        self.current_macrotask = "macro_task_0"
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_macrotask = rospy.Publisher("~macrotask", String, queue_size=1)
        # Setup subscriber
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self,msg):
        self.active = msg.data
        if self.active:
            self.publish_macrotask()

    def publish_macrotask(self):
        macro_msg = Int8()
        macro_msg.data = self.current_macrotask
        self.pub_macrotask.publish(macro_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = tsm_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()