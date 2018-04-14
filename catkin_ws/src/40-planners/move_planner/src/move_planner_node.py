#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import BoolStamped, Pose2DStamped, FSMState

class MovePlanner(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.active = False
        self.fsm_state = "JOYSTICK_CONTROL"

        # Path buffer
        self.moves = ['sleep']
        self.move = self.moves.pop()

        # Setup publishers
        self.pub_move = rospy.Publisher("~move",String, queue_size=1)
        self.pub_finish = rospy.Publisher("~finish", BoolStamped, queue_size=1)
        #self.pub_enter_planning = rospy.Publisher("~enter_planning", BoolStamped, queue_size=1)
        self.pub_set_confirm = rospy.Publisher("~confirm", BoolStamped, queue_size=1)

        # Setup subscriber
        self.sub_complete = rospy.Subscriber("~complete", BoolStamped, self.cbNextmove) # from communication
        self.sub_at_goal = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        self.sub_set_move = rospy.Subscriber("~set_move", String, self.cbSetMove)
        self.sub_fsm_state = rospy.Subscriber("/Robo/fsm_node/state", FSMState, self.cbState)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.5)
        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data
        if self.active:
            self.publishMove()
    
    def cbState(self, msg):
        self.fsm_state = msg.state

    def cbSetMove(self, msg):
        # Convert move from message
        moves = []
        moves.append(msg.data)
        rospy.loginfo("[%s] set move: %s" %(self.node_name, msg.data))
        # Set current move
        self.moves = moves
        self.move = self.moves.pop()
        # Publish confirm message
        conf_msg = BoolStamped()
        conf_msg.header.stamp = rospy.Time.now()
        self.pub_set_confirm.publish(conf_msg)

    def publishMove(self):     
        # Publish move message
        move_msg = String()
        move_msg.data = self.move
        self.pub_move.publish(move_msg)
        if self.move == 'sleep':
            rospy.sleep(5)
        rospy.loginfo("[%s] send move: %s" %(self.node_name, self.move))
        # Publish confirm message to task planner
        conf_msg = BoolStamped()
        conf_msg.header.stamp = rospy.Time.now()
        conf_msg.data = True
        self.pub_finish.publish(conf_msg)
        # Publish message to fsm enter planning state
        #rospy.sleep(2)
        #self.pub_enter_planning.publish(conf_msg)
        
    def cbNextmove(self,msg):
        if not self.active:
            return
        # Pop next move 
        try:
            # If we can pop another move
            self.move = self.moves.pop()
        except:
            # Cannot pop another move, reach destination
            reach_msg = BoolStamped()
            reach_msg.header.stamp = rospy.Time.now()
            reach_msg.data = True

    def cbTimer(self,event):
        if not self.active:
            return
        if self.fsm_state == "AT_GOAL":
            self.publishMove()
        # Set a timer to publish move
        if self.move == 'sleep':
            rospy.loginfo("[%s] sleeping" %(self.node_name))
            return 

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('move_planner_node', anonymous=False)

    # Create the NodeName object
    node = MovePlanner()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()