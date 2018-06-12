#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty

class Tester(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup test signal list
        self.signals = ['joy_off', 'reach_goal', 'reach_goal', 'reach_goal', 'reach_goal', 'reach_goal', 'reach_goal']

        # Read parameters
        self.timestep = self.setupParameter("~timestep",3.0)

        # Setup service provider
        self.start_test = rospy.Service('~start_test', Empty, self.cbSrvStart)

        # Setup services caller
        self.joy_on = rospy.ServiceProxy('/Robo/joy_mapper_node/set_joy', Empty)
        self.joy_off = rospy.ServiceProxy('/Robo/joy_mapper_node/joy_off', Empty)
        self.reach_goal = rospy.ServiceProxy('/Robo/odo_control_node/reach_goal', Empty)
        
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSrvStart(self, req):
        # Start test by setting a series of timers
        self.test_pointer = 0
        self.test_timer = rospy.Timer(rospy.Duration.from_sec(self.timestep), self.test, oneshot=True)
        return EmptyResponse()

    def test(self, event):
        signal = self.signals[self.test_pointer]
        self.test_pointer += 1
        if signal == 'joy_on':
            self.joy_on()
        elif signal == 'joy_off':
            self.joy_off()
        elif signal == 'reach_goal':
            self.reach_goal()
        else:
            return
        if self.test_pointer < len(self.signals):
            self.test_timer = rospy.Timer(rospy.Duration.from_sec(self.timestep), self.test, oneshot=True)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('tester_node', anonymous=False)

    # Create the NodeName object
    node = Tester()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()