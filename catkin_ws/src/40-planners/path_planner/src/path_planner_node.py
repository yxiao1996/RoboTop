#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import BoolStamped, Pose2DStamped, Pose2DList, FSMState

class PathPlanner(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.active = False
        self.fsm_state = "JOYSTICK_CONTROL"

        # Path buffer
        self.path = [(0.0, 0.0, 0.0), (1000.0, 0.0, 0.0), (1000.0, 1000.0, 0.0), (0.0, 1000.0, 0.0)]
        self.goal = self.path.pop()

        self.Rate = rospy.Rate(0.5)

        # Setup publishers
        self.pub_goal = rospy.Publisher("~goal",Pose2DStamped, queue_size=1)
        self.pub_reach_dest = rospy.Publisher("~reach_dest", BoolStamped, queue_size=1)
        self.pub_set_confirm = rospy.Publisher("~confirm", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_reach_goal = rospy.Subscriber("~reach_goal", BoolStamped, self.cbNextGoal)
        self.sub_set_path = rospy.Subscriber("~set_path", Pose2DList, self.cbSetPath)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.5)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data
        goal_msg = Pose2DStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.x = self.goal[0]
        goal_msg.y = self.goal[1]
        goal_msg.theta = self.goal[2]
        self.pub_goal.publish(goal_msg)

    def cbSetPath(self, msg):
        # Convert path from message
        path = []
        data_list = msg.data_list
        for i in range(len(data_list)):
            ith_pose = data_list[i]
            path.append((ith_pose.x, ith_pose.y, ith_pose.theta))
            rospy.loginfo("[%s] set path point: %s" %(self.node_name, path[i]))
        # Set current path
        self.path = path
        self.goal = self.path.pop()
        # Publish confirm message
        conf_msg = BoolStamped()
        conf_msg.header.stamp = rospy.Time.now()
        self.pub_set_confirm.publish(conf_msg)

    def cbNextGoal(self,msg):
        debug = False
        # Pop next goal on path
        try:
            # If we can pop another goal
            rospy.loginfo("[%s] reach destination, pop next goal"%(self.node_name))
            self.goal = self.path.pop()
            if not debug:
                goal_msg = Pose2DStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.x = self.goal[0]
                goal_msg.y = self.goal[1]
                goal_msg.theta = self.goal[2]
                self.pub_goal.publish(goal_msg)
            #self.Rate.sleep()
        except:
            # Cannot pop another goal, reach destination
            if debug:
                self.path = [(0.0, 0.0, 0.0), (1000.0, 0.0, 0.0), (1000.0, 1000.0, 0.0), (0.0, 1000.0, 0.0)]
            else:
                reach_msg = BoolStamped()
                reach_msg.header.stamp = rospy.Time.now()
                reach_msg.data = True
                self.pub_reach_dest.publish(reach_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('path_planner_node', anonymous=False)

    # Create the NodeName object
    node = PathPlanner()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
