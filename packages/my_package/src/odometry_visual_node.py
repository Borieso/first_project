#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from duckietown.dtros import DTROS, NodeType

class OdometryVisualizerNode(DTROS):
    def __init__(self, node_name):
        super(OdometryVisualizerNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        self.vehicle_name = rospy.get_param('~vehicle_name', 'duck1')
        self.topic = f"/{self.vehicle_name}/deadreckoning_node/odom"

        # Publishers
        self.marker_pub = rospy.Publisher(f"/{self.vehicle_name}/odom_path_marker", Marker, queue_size=10)

        # Pose history for path visualization
        self.path_points = []

        # Subscriber
        self.sub = rospy.Subscriber(self.topic, Odometry, self.odom_callback)

        rospy.loginfo(f"Subscribed to: {self.topic}")

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        # Save pose to draw trajectory line
        self.path_points.append(Point(pos.x, pos.y, pos.z))

        # 1. Publish arrow marker (orientation)
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "orientation_arrow"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.scale.x = 0.2  # shaft length
        arrow_marker.scale.y = 0.4
        arrow_marker.scale.z = 0.2
        arrow_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # red
        arrow_marker.points = [
            Point(pos.x, pos.y, pos.z),
            Point(pos.x + 0.5 * math.cos(yaw), pos.y + 0.5 * math.sin(yaw), pos.z)
        ]

        # 2. Publish line strip for path
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = "trajectory_line"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # blue
        path_marker.points = self.path_points

        # Publish both markers
        self.marker_pub.publish(arrow_marker)
        self.marker_pub.publish(path_marker)

if __name__ == '__main__':
    node = OdometryVisualizerNode(node_name='odom_visualizer_node')
    rospy.spin()
