#!/usr/bin/env python
import rospy

import numpy as np

#Because of transformations
from tf import transformations

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
  
    rospy.init_node('my_static_tf2_broadcaster')

    # tf2 broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "camera_link"
  
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 1.0
  
    # pitch, raw, yaw
    quat = transformations.quaternion_from_euler(np.pi*0.5, np.pi, 0.0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
 
    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()

