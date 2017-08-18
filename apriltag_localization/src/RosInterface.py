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
        # Internal variables
        self._no_detection = True
        self._t = None
        self._R = None
        #
        self._R_cam2bot = np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        self._t_cam2bot = np.concatenate((t_cam_to_body,np.array([[1]])), axis=0)
        #
        # self._R_tag2bot = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])

        # TODO: change the following hard coded camera name.
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


    def _tag_pose_callback(self, posearray):
        """
        Callback function for AprilTag measurements
        """
        if (len(posearray.detections)==0):
            return

        # TODO: Modify the dollowing line for multiple tag detection
        (self._t, self._R) = get_t_R(posearray.detections[0].pose.pose)

        # TODO: Modify the following line if the tag is pasted on the ceil.
        #-----------------------------#
        self._angle = -np.arctan2(-self._R[2,0],np.sqrt(self._R[2,0]**2+self._R[2,2]**2))
        if math.isnan(self._angle):
            return
        #-----------------------------#

        # self._R = np.dot(np.dot(self._R_cam2bot, self._R),self._R_tag2bot)
        self._t = np.dot(self._R_cam2bot, self._t) + self._t_cam2bot

        self._marker_num = posearray.detections[0].id
        self._no_detection = False

    def get_measurements(self):
        """
        Returns information about the last tag seen if any. Returns (x,y,theta) as a
        3x1 numpy array. Returns None if no new tag is seen.
        """
        if self._no_detection:
            return None
        self._no_detection = True
        dx = self._t[0,0]
        dy = self._t[1,0]
        return [[dx,dy,self._angle,self._marker_num]]

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
