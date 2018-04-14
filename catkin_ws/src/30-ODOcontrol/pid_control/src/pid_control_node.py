#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from robocon_msgs.msg import BoolStamped, Pose2DStamped, Twist2DStamped
from dynamic_reconfigure.server import Server
from pid_control.cfg import pidConfig

def cbConfigure(config, level):
    rospy.loginfo("""Reconfigure Request: {kp}, {ki}, {kd}""".format(**config))
    return config

class PIDcontrolnode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # set state buffer
        self.active = False

        # Setup publishers
        self.pub_twist2d = rospy.Publisher("~twist2d",Twist2DStamped, queue_size=1)
        # Setup subscriber
        self.cur_pos_update = rospy.Subscriber("~cur_pos_update", Pose2DStamped, self.pidUpdate)
        self.ref_pos_update = rospy.Subscriber("~ref_pos_update", Pose2DStamped, self.setAimPos)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.05)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)
        # Initialize aim position
        self.aim_pos_x = 0.0
        self.aim_pos_y = 0.0
        self.aim_pos_theta = 0.0
        # PID parameters
        self.kp = -0.01
        self.ki = 0.0
        self.kd = 0.0
        self.intg_len = 100#actual time length based on position refresh rate
        self.intg_max = 0.1
        self.intg_data_x = []
        self.intg_data_y = []
        self.intg_data_theta = []
        self.max_output_v = 0.5
        self.max_output_omega = 0.5
        # Output data
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        # rqt reconfigure
        self.srv = Server(pidConfig, self.cbConfigure)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data

    def pidUpdate(self,msg):
        if self.active == False:
            return
        #rospy.loginfo("[%s] %s" %(self.node_name,msg.data))
	if msg.x == 0.0 and msg.y == 0.0 and msg.theta == 0.0:
	    return
        # Error
        err_x = msg.x - self.aim_pos_x
        err_y = msg.y - self.aim_pos_y
        err_theta = msg.theta - self.aim_pos_theta
        # Integration value constrain
        intg_x_vule = self.ki*sum(self.intg_data_x)/self.intg_len
        intg_y_vule = self.ki*sum(self.intg_data_y)/self.intg_len
        intg_theta_vule = self.ki*sum(self.intg_data_theta)/self.intg_len
        intg_x_vule = self.constrain(intg_x_vule,self.intg_max)
        intg_y_vule = self.constrain(intg_y_vule,self.intg_max)
        intg_theta_vule = self.constrain(intg_theta_vule,self.intg_max)
        # PID calculation
        vx_temp = self.kp*err_x + intg_x_vule
        vy_temp = self.kp*err_y + intg_y_vule
        omega_temp = self.kp*err_theta + intg_theta_vule
        # PID output constrain
        self.vx = self.constrain(vx_temp,self.max_output_v)
        self.vy = self.constrain(vy_temp,self.max_output_v)
        self.omega = self.constrain(omega_temp,self.max_output_v)
        # Integration arrays update
        self.intg_data_x.append(err_x)
        if len(self.intg_data_x) > self.intg_len: #Constrain integration length
            self.intg_data_x.pop(0)
        self.intg_data_y.append(err_y)
        if len(self.intg_data_y) > self.intg_len:
            self.intg_data_y.pop(0)
        self.intg_data_theta.append(err_theta)
        if len(self.intg_data_theta) > self.intg_len:
            self.intg_data_theta.pop(0)
        
    def constrain(self,input,max):
        if abs(input)>max:
            if input>0:
                return abs(max)
            else:
                return -abs(max)
        else:
            return input

        
    def setAimPos(self,msg):
        #if self.active == False:
        #    return
        self.aim_pos_x = msg.x
        self.aim_pos_y = msg.y
        self.aim_pos_theta = msg.theta
        #rospy.loginfo("Aim position has updated as:\n x=%5.4f, y=%5.4f, theta=%5.4f" %(msg.x,msg.y,msg.theta))

    def cbTimer(self,event):
        if self.active == False:
            return
        #singer = HelloGoodbye()
        # Simulate hearing something
        msg = Twist2DStamped()
        #msg.data = singer.sing("duckietown")
        msg.v_x = self.vx
        msg.v_y = self.vy
        msg.omega = self.omega
        self.pub_twist2d.publish(msg)

    def cbConfigure(self, config, level):
        rospy.loginfo("""Reconfigure Request: kp: {kp}, ki: {ki}, kd: {kd}""".format(**config))
        self.kp = config['kp']
        self.ki = config['ki']
        self.kd = config['kd']
        self.intg_len = config['intg_len']
        self.intg_max = config['intg_max']
        self.max_output_v = config['max_output_v']
        self.max_output_omega = config['max_output_omega']
        print self.kp, self.ki, self.kd, self.intg_len, self.intg_max, self.max_output_v, self.max_output_omega

        return config

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('PIDcontrolnode', anonymous=False)

    # Create the NodeName object
    node = PIDcontrolnode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
