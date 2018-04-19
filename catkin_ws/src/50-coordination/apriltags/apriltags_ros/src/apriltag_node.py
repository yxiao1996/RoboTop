#!/usr/bin/env python
import tf
import rospkg
import rospy
import yaml
from robocon_msgs.msg import AprilTagDetectionArray
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class apriltag_node(object):
    def __init__(self):
        self.node_name = "apriltag_node"

        self.listener = tf.TransformListener()

        # Load parameters
        self.camera_x     = self.setupParam("~camera_x", 0.065)
        self.camera_y     = self.setupParam("~camera_y", 0.0)
        self.camera_z     = self.setupParam("~camera_z", 0.11)
        self.camera_theta = self.setupParam("~camera_theta", 19.0)
        self.scale_x     = self.setupParam("~scale_x", 1)
        self.scale_y     = self.setupParam("~scale_y", 1)
        self.scale_z     = self.setupParam("~scale_z", 1)

        self.sub_prePros = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        
        tag_infos = []

         # Load tag detections message
        for detection in msg.detections:
            tag_id = int(detection.id)

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

            frame_name = '/carrot'+str(tag_id)
            try:
                (trans_tf,rot_tf) = self.listener.lookupTransform('/turtle1', frame_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            rot_tf_rpy = tf.transformations.euler_from_quaternion(rot_tf)
            yaw = rot_tf_rpy[2] / 3.14

            print rot.z - yaw
            #print tag_id, trans, rot



if __name__ == '__main__': 
    rospy.init_node('apriltag_node',anonymous=False)
    node = apriltag_node()
    rospy.spin()