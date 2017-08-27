#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
# ROS imports
import roslib
import rospy

from std_msgs.msg import (
    Header,
)

from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection,
)


from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Pose,
    Quaternion,
    PoseWithCovariance,
)

import cv2
import yaml
import numpy as np

import sys

import tf
from tf.transformations import *

# Extra utility functions
from utility import *

class ROSInterface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, camera_frame_id, t_cam_to_body):
        """
        Initialize the class
        """
        # Is the camera installed on top or at front-side?
        self.camera_top_and_tag_top = True

        # Internal variables
        self._no_detection = True
        self.num_detections = 0
        self._t = list() # None
        self._R = list() # None
        self.angle = list() # None
        self._marker_num = list() # None

        # _R_tag_2_tag1: rotate +180 deg about x-axis
        # self._R_tag_2_tag1 = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        if self.camera_top_and_tag_top:
            # Camera installed on the top
            self._R_cam2bot = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
        else:
            # Camera installed at fron-side
            self._R_cam2bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        #
        self._t_cam2bot = np.concatenate((t_cam_to_body,np.array([[1]])), axis=0)
        # self._R_tag2bot = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])

        #
        self.camera_frame_id = camera_frame_id
        # camera_frame_id = "usb_cam"
        # ROS publishers and subscribers
        rospy.Subscriber("/%s/tag_detections" % camera_frame_id, AprilTagDetectionArray,self._tag_pose_callback)
        # amcl
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_CB)
        self._pub_init_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        #
        self._amcl_poseStamp = None # Header()
        self._amcl_pose = None # Pose()
        self._amcl_cov = None # np.eye(6)
        #
        self._T_subState = np.zeros((3,6))
        self._T_subState[0,0] = 1.0
        self._T_subState[1,1] = 1.0
        self._T_subState[2,5] = 1.0

        # tf
        self.tf_listener = tf.TransformListener()


    def _tag_pose_callback(self, posearray):
        """
        Callback function for AprilTag measurements
        """
        num_detections = len(posearray.detections)
        if (num_detections == 0):
            return

        #
        self._R = list()
        self._t = list()
        self._angle = list()
        self._marker_num = list()

        # Multiple tags detection
        for kk in range(num_detections):
            # Get id
            _marker_num = posearray.detections[kk].id
            # Get R and t
            # _t_tag_2_cam is in camera fram
            (_t_tag_2_cam, _R_tag_2_cam) = get_t_R(posearray.detections[kk].pose.pose)
            #-----------------------------#
            _angle_tag_2_bot = self.get_angle_tag_2_cam_from_R(_R_tag_2_cam)
            if _angle_tag_2_bot is None:
                return
            # _R_tag_2_bot = np.dot(np.dot(self._R_cam2bot, _R_tag_2_cam),self._R_tag2bot)
            # _R_tag_2_bot = np.dot(self._R_cam2bot, _R_tag_2_cam)
            _t_tag_2_bot = np.dot(self._R_cam2bot, _t_tag_2_cam) + self._t_cam2bot
            #-----------------------------#
            #
            self._R.append(_R_tag_2_cam)
            self._t.append(_t_tag_2_bot)
            self._angle.append(_angle_tag_2_bot)
            self._marker_num.append(_marker_num)
        #
        self._no_detection = False
        self.num_detections = num_detections

    def get_angle_tag_2_cam_from_R(self, _R_tag_2_cam):
        # Modify the following lines if the tag is pasted on the ceil.
        if self.camera_top_and_tag_top:
            # Case 1: The tags are pasted on the ceil
            # _R_tag1_2_cam = self._R_tag_2_tag1.dot(_R_tag_2_cam)
            _angle_tag_2_bot = np.arctan2(_R_tag_2_cam[1,0], _R_tag_2_cam[0,0])
        else:
            # Case 2: The tags are pasted on the wall
            _angle_tag_2_bot = -np.arctan2(-_R_tag_2_cam[2,0], np.sqrt(_R_tag_2_cam[2,0]**2 + _R_tag_2_cam[2,2]**2))

        # Prevent the sigularity
        if math.isnan(_angle_tag_2_bot):
            return None
        else:
            return _angle_tag_2_bot



    def get_measurements(self):
        """
        Returns information about the last tag seen if any. Returns a list of Python list
        in the format of (x,y,theta,id). Returns None if no new tag is seen.
        """
        if self._no_detection:
            return None
        self._no_detection = True

        # Note all tags are represented in robot's coordinate
        tag_list = list()
        for kk in range(len(self._t)): # range(self.num_detections):
            dx = self._t[kk][0,0]
            dy = self._t[kk][1,0]
            tag_list.append([dx,dy,self._angle[kk],self._marker_num[kk]])
        return tag_list

    def get_measurements_tf(self):
        """
        Returns information about the last tag seen if any. Returns a list of Python list
        in the format of (x,y,theta,id). Returns None if no new tag is seen.
        """
        if self._no_detection:
            return None
        # self._no_detection = True

        # Note all tags are represented in robot's coordinate
        camera_frame = "/%s" % self.camera_frame_id
        tag_list = list()
        for kk in range(len(self._marker_num)): # range(self.num_detections):
            #
            tagFramName = "/tag_%d" % self._marker_num[kk]
            try:
                # From /usb_cam to a tag
                (_t_cam_2_tag, quaternion) = self.tf_listener.lookupTransform( camera_frame, tagFramName, rospy.Time(0))
            except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            _R_tag_2_cam = quaternion_matrix(quaternion)
            _angle_tag_2_bot = self.get_angle_tag_2_cam_from_R(_R_tag_2_cam)
            if _angle_tag_2_bot is None:
                continue
            _t_tag_2_bot = np.dot(self._R_cam2bot, _t_cam_2_tag) + self._t_cam2bot
            dx = _t_tag_2_bot[0,0]
            dy = _t_tag_2_bot[1,0]
            tag_list.append([dx,dy,_angle_tag_2_bot,self._marker_num[kk]])
        #
        self._no_detection = True
        return tag_list


    def amcl_pose_CB(self, PoseWithCovarianceStamped):
        self._amcl_poseStamp = PoseWithCovarianceStamped.header
        self._amcl_pose = PoseWithCovarianceStamped.pose.pose
        self._amcl_cov = np.array(PoseWithCovarianceStamped.pose.covariance).reshape(6,6)

    def get_amcl_pose(self):
        if self._amcl_poseStamp is None:
            return None
        else:
            # [x, y, theta].'
            pose_2D = np.zeros((3,1))
            pose_2D[0,0] = self._amcl_pose.position.x
            pose_2D[1,0] = self._amcl_pose.position.y
            #
            pose = self._amcl_pose
            quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # roll = euler[0]
            # pitch = euler[1]
            yaw = euler[2]
            pose_2D[2,0] = yaw
            # Reduced-order covariance matrix
            cov_2D = np.dot(np.dot(self._T_subState, self._amcl_cov), np.transpose(self._T_subState) )
            #
            return (pose_2D, cov_2D)

    def get_amcl_pose_tf(self):
        # Get pose_2D from tf to reduce the effect of delay
        try:
            # From /map to /base_footprint
            (trans, quaternion) = self.tf_listener.lookupTransform('/map','/base_footprint', rospy.Time(0))
            #
            """
            now = rospy.Time.now()
            self.tf_listener.waitForTransform('/map','/base_footprint', now, rospy.Duration(1.0))
            (trans, quaternion) = self.tf_listener.lookupTransform('/map','/base_footprint', now)
            """
        except: # (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        #
        if self._amcl_poseStamp is None:
            return None
        else:
            # [x, y, theta].'
            pose_2D = np.zeros((3,1))
            #
            pose_2D[0,0] = trans[0] # self._amcl_pose.position.x
            pose_2D[1,0] = trans[1] # self._amcl_pose.position.y
            #
            # pose = self._amcl_pose
            # quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            # roll = euler[0]
            # pitch = euler[1]
            yaw = euler[2]
            pose_2D[2,0] = yaw
            # Reduced-order covariance matrix
            cov_2D = np.dot(np.dot(self._T_subState, self._amcl_cov), np.transpose(self._T_subState) )
            #
            return (pose_2D, cov_2D)

    def set_amcl_pose(self, pose_2D, cov_2D):
        #
        t = [pose_2D[0,0], pose_2D[1,0], 0.0]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, pose_2D[2,0]) # raw, pitch, yaw
        #
        Cov_np = np.dot( np.dot( np.transpose(self._T_subState), cov_2D), self._T_subState )
        Cov = Cov_np.reshape(1,36).tolist()[0] # Convert to a python list (1-D)
        #
        pose_cov_stamped_msg = make_pose_covariance_stamped_msg_quat(t, quaternion, Cov)
        # Publish
        self._pub_init_amcl.publish(pose_cov_stamped_msg)
