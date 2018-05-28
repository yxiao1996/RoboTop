#!/usr/bin/env python
import rospy
from std_msgs.msg import String, ColorRGBA #Imports msg
from robocon_msgs.msg import BoolStamped, Pose2DStamped, Twist2DStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class RobotVisualizer(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        
        # Setup publishers
        # self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        self.pub_position = rospy.Publisher("~position", MarkerArray, queue_size=1)
        #self.pub_seg_list = rospy.Publisher("~segment_list_markers",MarkerArray,queue_size=1)

        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        #self.seg_color_dict = dict()
        #self.seg_color_dict[Segment.WHITE] = ColorRGBA(r=1.0,g=1.0,b=1.0,a=1.0)
        #self.seg_color_dict[Segment.YELLOW] = ColorRGBA(r=1.0,g=1.0,b=0.0,a=1.0)
        #self.seg_color_dict[Segment.RED] = ColorRGBA(r=1.0,g=0.0,b=0.0,a=1.0)

        # Setup subscriber
        self.sub_pose = rospy.Subscriber("/Robo/serial_decoder_node/odo_msg", Pose2DStamped, self.cbPose, queue_size=1)
        #self.sub_seg_list = rospy.Subscriber("~segment_list", SegmentList, self.cbSegList)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def cbPose(self, msg):
        x = msg.x
        y = msg.y
        #print x, y
        points = Marker()
        points.header.frame_id = 'turtle1'
        points.header.stamp = rospy.Time.now()
        points.ns = 'turtle1/position'
        points.action = Marker.ADD
        points.lifetime = rospy.Duration.from_sec(5.0)
        points.type = Marker.POINTS
        points.scale.x = 0.1
        points.scale.y = 0.1
        points.color.g = 1.0
        points.color.a = 1.0
        p = Point()
        p.x = x / 1000.0
        p.y = y / 1000.0
        p.z = 0.0
        points.points.append(p)

        marker_array = MarkerArray()
        marker_array.markers.append(points)
        self.pub_position.publish(marker_array)


    def cbSegList(self,seg_list_msg):
        marker_array = MarkerArray()
        marker_array.markers.append(self.segList2Marker(seg_list_msg))
        # rospy.loginfo("[%s] publishing %s marker."%(self.node_name,len(marker_array.markers)))
        self.pub_seg_list.publish(marker_array)

    def segList2Marker(self,seg_list_msg):
        marker = Marker()
        marker.header.frame_id = self.veh_name
        marker.header.stamp = seg_list_msg.header.stamp
        marker.ns = self.veh_name + "/line_seg"
        marker.id = 0
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(5.0)
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        for seg in seg_list_msg.segments:
            # point_start = seg.points[0]
            # point_end = seg.points[1]
            marker.points.append(seg.points[0])
            marker.points.append(seg.points[1])
            color = self.seg_color_dict[seg.color]
            marker.colors.append(color)
            marker.colors.append(color)

        # rospy.loginfo("[%s] Number of points %s" %(self.node_name,len(marker.points)))
        return marker

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('robot_visualizer', anonymous=False)

    # Create the NodeName object
    node = RobotVisualizer()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()