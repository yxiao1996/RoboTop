#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray#Imports msg
from robocon_msgs.msg import BoolStamped, Pose2DList, Pose2DStamped

class task_planner_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Task buffer
        self.Task = [{'path': [(0.0, 0.0,  0.0)],
                      'move': 'sleepforever'},
                     {'path': [(1000.0, 1000.0, 0.0), (0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)],
                      'move': 'sleep'},
                     {'path': [(2000.0, 1000.0, 0.0), (2000.0, 0.0, 0.0), (1000.0, 0.0, 0.0)],
                      'move': 'sleep'}]
        self.current_task = self.Task.pop()
        self.default_task = {'path': [(0.0, 0.0,  0.0)],
                             'move': 'sleepforever'}

        # Status buffer
        self.active = False
        self.path_ready = False
        self.move_ready = False
        self.path_finish = False
        self.move_finish = False

        # Setup publishers
        self.pub_confrim = rospy.Publisher("~confirm", BoolStamped, queue_size=1)
        self.pub_finish = rospy.Publisher("~finish", BoolStamped, queue_size=1)
        self.pub_set_path = rospy.Publisher("~set_path", Pose2DList, queue_size=1)
        self.pub_set_move = rospy.Publisher("~set_move", String, queue_size=1)
        # Setup subscriber
        self.sub_confirm_path = rospy.Subscriber("~confirm_path", BoolStamped, self.cbConfPath)
        self.sub_confirm_move = rospy.Subscriber("~confirm_move", BoolStamped, self.cbConfMove)
        self.sub_finish_path = rospy.Subscriber("~finish_path", BoolStamped, self.cbFinishPath)
        self.sub_finish_move = rospy.Subscriber("~finish_move", BoolStamped, self.cbFinishMove)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
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
        if self.active:
            self.set_task()

    def cbConfMove(self,msg):
        # Confrim set move
        self.move_ready = msg.data
        rospy.loginfo("[%s] move setting success" %(self.node_name))
        self.move_ready = True
        if self.path_ready and self.move_ready:
            self.pub_confrim_msg()

    def cbConfPath(self,msg):
        # Confirm set path
        self.path_ready = msg.data
        rospy.loginfo("[%s] path setting success" %(self.node_name))
        self.path_ready = True
        if self.move_ready and self.path_ready:
            self.pub_confrim_msg()

    def cbFinishMove(self, msg):
        # Confirm finish move
        self.move_finish = msg.data
        rospy.loginfo("[%s] move finished" %(self.node_name))
        self.move_finish = True
        if self.move_finish and self.path_finish:
            self.next_task()

    def cbFinishPath(self, msg):
        # Confirm finish move
        self.path_finish = msg.data
        rospy.loginfo("[%s] path finished" %(self.node_name))
        self.path_finish = True
        if self.move_finish and self.path_finish:
            self.next_task()        

    def next_task(self):
        # Pop next task
        try:
            self.current_task = self.Task.pop()
            rospy.loginfo("[%s] pop next task" %(self.node_name))
            # Publish finish message
            finish_msg = BoolStamped()
            finish_msg.header.stamp = rospy.Time.now()
            finish_msg.data = True
            self.pub_finish.publish(finish_msg)
        except:
            # Publish finish message
            finish_msg = BoolStamped()
            finish_msg.header.stamp = rospy.Time.now()
            finish_msg.data = True
            self.pub_finish.publish(finish_msg)
            debug = True
            if debug:
                self.current_task = self.default_task
                rospy.loginfo("[%s] pop default task" %(self.node_name))

    def pub_confrim_msg(self):
        # Finish planning, set fsm to next state
        finish_msg = BoolStamped()
        finish_msg.header.stamp = rospy.Time.now()
        finish_msg.data = True
        self.pub_confrim.publish(finish_msg)
        # set status buffer
        self.move_ready = False
        self.path_ready = False

    def set_task(self):
        path_msg = Pose2DList()
        move_msg = String()

        # Publish move message
        move_msg.data = self.current_task['move']
        self.pub_set_move.publish(move_msg)

        # Publish path messgae
        data_list = []
        for i in range(len(self.current_task['path'])):
            # ith position
            data = self.current_task['path'][i]
            ith_pose = Pose2DStamped()
            ith_pose.header.stamp = rospy.Time.now()
            ith_pose.x = data[0]
            ith_pose.y = data[1]
            ith_pose.theta = data[2]
            data_list.append(ith_pose)
        path_msg.data_list = data_list
        self.pub_set_path.publish(path_msg)

        rospy.loginfo("[%s] set move and path to planners" %(self.node_name))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('task_planner_node', anonymous=False)

    # Create the NodeName object
    node = task_planner_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()