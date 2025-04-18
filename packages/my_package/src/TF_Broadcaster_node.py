#!/usr/bin/env python3

import os
import rospy
import tf
from nav_msgs.msg import Odometry
from duckietown.dtros import DTROS, NodeType

class OdometryTFBroadcaster(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OdometryTFBroadcaster, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # Get the vehicle name from environment variables
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'duckie1')
        
        # Construct subscriber to the odometry topic
        self._subscriber = rospy.Subscriber(f'/{self._vehicle_name}/odom', Odometry, self.odom_callback)
        
        # TF broadcaster
        self._tf_broadcaster = tf.TransformBroadcaster()
    
    def odom_callback(self, msg):
        # Broadcast the transform
        self._tf_broadcaster.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            rospy.Time.now(),
            msg.child_frame_id,   # "duckie1/base_footprint"
            msg.header.frame_id   # "duckie1/odom"
        )
    
if __name__ == '__main__':
    # Create the node
    node = OdometryTFBroadcaster(node_name='odom_tf_broadcaster')
    # Keep the process from terminating
    rospy.spin()
