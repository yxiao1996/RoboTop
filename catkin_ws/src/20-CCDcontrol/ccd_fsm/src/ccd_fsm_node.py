#!/usr/bin/env python
import rospy
import copy
#from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import FSMState, BoolStamped

class CCDFSM(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Build transition dictionary
        self.states_dict = rospy.get_param("~states", {})
        # Validate state and global transitions
        if not self._validateStates(self.states_dict):
            rospy.signal_shutdown("[%s] Incoherent definition." %self.node_name)
        
        # Load global transitions
        self.global_transitions_dict = rospy.get_param("~global_transitions", {})
        if not self._validateGlobalTransitions(self.global_transitions_dict,self.states_dict.keys()):
            rospy.signal_shutdown("[%s] Incoherent definition." %self.node_name)
            return

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = rospy.get_param("~initial_state", "")
        self.state_msg.header.stamp = rospy.Time.now()
        # Setup publishers
        self.pub_state = rospy.Publisher("~state",FSMState, queue_size=1)

        # Construct publishers
        self.pub_dict = dict()
        nodes = rospy.get_param("~nodes")

        self.active_nodes = None
        for node_name, topic_name in nodes.items():
            self.pub_dict[node_name] = rospy.Publisher(topic_name, BoolStamped, queue_size=1, latch=True)
        
        # Process events definition
        param_events_dict = rospy.get_param("~events", {})
        if not self._validateEvents(param_events_dict):
            rospy.signal_shutdown("[%s] Invalid event definition." %self.node_name)
            return   

        # Setup subscriber
        self.sub_list = list()
        self.event_trigger_dict = dict()
        for event_name, event_dict in param_events_dict.items():
            topic_name = event_dict["topic"]
            msg_type = event_dict["msg_type"]
            self.event_trigger_dict[event_name] = event_dict["trigger"]
            self.sub_list.append(rospy.Subscriber("%s"%(topic_name), BoolStamped, self.cbEvent, callback_args=event_name))
            
        rospy.loginfo("[%s] Initialzed." %(self.node_name))
        # Publish initial state
        self.publish()
    
    def _validateStates(self,states_dict):
        pass_flag = True
        valid_states = states_dict.keys()
        for state, state_dict in states_dict.items():        
            # Validate the existence of all reachable states
            transitions_dict = state_dict.get("transitions")
            if transitions_dict is None:
                continue
            else:
                for transition, next_state in transitions_dict.items():
                    if next_state not in valid_states:
                        rospy.logerr("[%s] %s not a valide state. (From %s with event %s)" %(self.node_name,next_state,state,transition))
                        pass_flag = False
        return pass_flag 

    def _validateGlobalTransitions(self,global_transitions, valid_states):
        pass_flag = True
        for event_name, state_name in global_transitions.items():
            if state_name not in valid_states:
                rospy.logerr("[%s] State %s is not valid. (From global_transitions of %s)" %(self.node_name,state_name, event_name)) 
                pass_flag = False
        return pass_flag

    def _validateEvents(self,events_dict):
        pass_flag = True
        for event_name, event_dict in events_dict.items():
            print 
            if "topic" not in event_dict:
                rospy.logerr("[%s] Event %s missing topic definition." %(self.node_name,event_name))
                pass_flag = False
            if "msg_type" not in event_dict:
                rospy.logerr("[%s] Event %s missing msg_type definition." %(self.node_name,event_name))
                pass_flag = False
            if "trigger" not in event_dict:
                rospy.logerr("[%s] Event %s missing trigger definition." %(self.node_name,event_name))
                pass_flag = False
        return pass_flag

    def _getNextState(self, state_name, event_name):
        if not self.isValidState(state_name):
            rospy.logwarn("[%s] %s not defined. Treat as terminal. "%(self.node_name,state_name))
            return None

        # state transitions overwrites global transition
        state_dict = self.states_dict.get(state_name)
        if "transitions" in state_dict:
            next_state = state_dict["transitions"].get(event_name)
        else:
            next_state = None

        # state transitions overwrites global transitions
        if next_state is None:
            # No state transition defined, look up global transition
            next_state = self.global_transitions_dict.get(event_name) #None when no global transitions
        return next_state
                
    def _getActiveNodesOfState(self,state_name):
        state_dict = self.states_dict[state_name]
        active_nodes = state_dict.get("active_nodes")
        if active_nodes is None:
            rospy.logwarn("[%s] No active nodes defined for %s. Deactive all nodes."%(self.node_name,state_name))
            active_nodes = []
        return active_nodes
    
    def isValidState(self,state):
        return state in self.states_dict.keys()

    def publish(self):
        self.publishBools()
        self.publishState()

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        rospy.loginfo("[%s] FSMState: %s" %(self.node_name, self.state_msg.state))

    def publishBools(self):
        active_nodes = self._getActiveNodesOfState(self.state_msg.state)
        for node_name, node_pub in self.pub_dict.items():
            msg = BoolStamped()
            msg.header.stamp = self.state_msg.header.stamp
            msg.data = bool(node_name in active_nodes)
            node_state = "ON" if msg.data else "OFF"
            rospy.loginfo("[%s] Node %s is %s in %s" %(self.node_name, node_name, node_state, self.state_msg.state))
            if self.active_nodes is not None:
                if (node_name in active_nodes) == (node_name in self.active_nodes):
                    continue
            # else:
            #     rospy.logwarn("[%s] self.active_nodes is None!" %(self.node_name))
                # continue
            node_pub.publish(msg)
            # rospy.loginfo("[%s] node %s msg %s" %(self.node_name, node_name, msg))
            # rospy.loginfo("[%s] Node %s set to %s." %(self.node_name, node_name, node_state))
        self.active_nodes = copy.deepcopy(active_nodes)

    def cbEvent(self,msg,event_name):
        if (msg.data == self.event_trigger_dict[event_name]):
            # Update timestamp
            rospy.loginfo("[%s] Event: %s" %(self.node_name,event_name))
            self.state_msg.header.stamp = msg.header.stamp
            next_state = self._getNextState(self.state_msg.state,event_name)
            if next_state is not None:
                # Has a defined transition
                self.state_msg.state = next_state
                self.publish()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ccd_fsm_node', anonymous=False)

    # Create the NodeName object
    node = CCDFSM()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()