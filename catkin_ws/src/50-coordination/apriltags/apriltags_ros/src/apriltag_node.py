#!/usr/bin/env python
import tf
import rospkg
import rospy
import yaml
import numpy as np
from std_msgs.msg import Int8
from robocon_msgs.msg import AprilTagDetectionArray, BoolStamped
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class apriltag_node(object):
    def __init__(self):
        self.node_name = "apriltag_node"

        self.active = True

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # Load parameters
        self.camera_x     = self.setupParam("~camera_x", 0.065)
        self.camera_y     = self.setupParam("~camera_y", 0.0)
        self.camera_z     = self.setupParam("~camera_z", 0.11)
        self.camera_theta = self.setupParam("~camera_theta", 0.0) # 19
        self.scale_x     = self.setupParam("~scale_x", 1)
        self.scale_y     = self.setupParam("~scale_y", 1)
        self.scale_z     = self.setupParam("~scale_z", 1)

        self.sub_prePros = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.pub_tagId = rospy.Publisher("~tag_id", Int8, queue_size=1)
        self.pub_coord = rospy.Publisher("~coord", BoolStamped, queue_size=1)
        self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)
        self.pub_ack = rospy.Publisher("~ack_apriltag", BoolStamped, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self, msg):
        self.active = msg.data

    def callback(self, msg):
        if not self.active:
            return
        #print msg
        tag_infos = []
        trans_x_buf = 0.0
        trans_y_buf = 0.0
        trans_z_buf = 0.0
        rot_z_buf = 0.0
        yaw_buf = 0.0

         # Load tag detections message
        for detection in msg.detections:
            tag_id = int(detection.id)
            self.checkProtocol(tag_id)

            # Define the transforms
            veh_t_camxout = tr.translation_matrix((self.camera_x, self.camera_y, self.camera_z))
            veh_R_camxout = tr.euler_matrix(0, self.camera_theta*np.pi/180, 0, 'rxyz')
            veh_T_camxout = tr.concatenate_matrices(veh_t_camxout, veh_R_camxout)   # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rzyx')
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

            #Load translation
            trans = detection.pose.pose.position
            rot = detection.pose.pose.orientation

            camzout_t_tagzout = tr.translation_matrix((trans.x*self.scale_x, trans.y*self.scale_y, trans.z*self.scale_z))
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            frame_name = '/tag'+str(tag_id)
            try:
                (trans_tf,rot_tf) = self.listener.lookupTransform('/ducky1', frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rot_tf_rpy = tf.transformations.euler_from_quaternion(rot_tf)
            yaw = rot_tf_rpy[2] / np.pi
            # Check phase
            if tag_id == 3:
                # Back of the objet, phase confusion
                if rot.z < 0.0:
                    # turn around the coordinate frame to avoid confusion
                    yaw = -yaw

            # Publish tag id
            tag_id_msg = Int8()
            tag_id_msg.data = tag_id
            self.pub_tagId.publish(tag_id_msg)

            # Publish ack
            ack_msg = BoolStamped()
            if trans.x < 0.7:
                ack_msg.data = True
            else:
                ack_msg.data = False
            self.pub_ack.publish(ack_msg)
            
            #print detection
            print detection #trans.x, trans.y, trans.z
            #self.checkProtocol(tag_id)
            #print rot.z - yaw
            #print tag_id, trans, rot
            rot_z_buf += rot.z
            yaw_buf += yaw
        if len(msg.detections) > 0:
            # Broadcast tranform
            self.broadcaster.sendTransform((trans.x, trans.y, trans.z),
                                            tf.transformations.quaternion_from_euler(0, 0, np.pi*(rot_z_buf-yaw_buf)/float(len(msg.detections))),
                                            rospy.Time.now(),
                                            "ducky1",
                                            "camera")

    def checkProtocol(self, tag_id):
        # A fake protocol for test
        if tag_id == 1:
            task_success = True
        else:
            task_success = False
        coord_msg = BoolStamped()
        coord_msg.header.stamp = rospy.Time.now()
        coord_msg.data = task_success
        self.pub_coord.publish(coord_msg)

if __name__ == '__main__': 
    rospy.init_node('apriltag_node',anonymous=False)
    node = apriltag_node()
    rospy.spin()
