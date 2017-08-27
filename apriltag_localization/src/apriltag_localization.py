#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
from math import atan2
# Extra utility functions
from utility import *
#
from KalmanFilter import KalmanFilter



class ApriltagLocalization(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, camera_frame_id, world_map, pos_init, t_cam_to_body):
        """
        Initialize the class
        """
        # Handles all the ROS related items
        self.ros_interface = ROSInterface(camera_frame_id, t_cam_to_body)
        self.time = rospy.get_time()
        # YOUR CODE AFTER THIS
        #-------------------------------------------#
        self.markers = world_map
        self.idx_target_marker = 0
        #-------------------------------------------#
        # Kalman filter
        self.kalman_filter = KalmanFilter(world_map)
        self.kalman_filter.mu_est = pos_init # 3*pos_init # For test


    def process_measurements(self):
        """
        YOUR CODE HERE
        This function is called at 10Hz
        """
        # tag_measurement = self.ros_interface.get_measurements()
        tag_measurement = self.ros_interface.get_measurements_tf()
        # amcl_pose = self.ros_interface.get_amcl_pose()

        print '-----------------'
        print 'tag:'
        print tag_measurement

        #----------------------------------------#
        # self.time = rospy.get_time()
        # print "self.time",self.time

        # Kalman filter
        # self.kalman_filter.step_filter(self.controlOut[0], (-1)*imu_meas, tag_measurement, rospy.get_time())
        self.kalman_filter.step_filter_by_amcl(self.ros_interface, tag_measurement, rospy.get_time())
        """
        print "After--"
        print "mu_est",self.kalman_filter.mu_est
        print "angle_est =", (self.kalman_filter.mu_est[2,0]*180.0/np.pi), "deg"
        """
        #
        return

def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    #
    camera_frame_id = np.array(params['camera_frame_id'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    t_cam_to_body = np.array(params['t_cam_to_body'])

    #
    # camera_frame_id = "usb_cam"

    # Intialize the ApriltagLocalization object
    apriltag_localization = ApriltagLocalization(camera_frame_id, world_map, pos_init, t_cam_to_body)

    # Call process_measurements at 10Hz
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        apriltag_localization.process_measurements()
        r.sleep()
    # Done

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
