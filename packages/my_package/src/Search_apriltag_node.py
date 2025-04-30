#!/usr/bin/env python3

import os
import rospy
import time
from random import uniform
from random import randint
from std_msgs.msg import Bool  # Import Bool message type
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Transform, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf import transformations as tr
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform





# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 1.0     # angular vel   , rad/s     , counter clock wise (+)


class SearchApriltagNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(SearchApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.reset_odom_topic = f"{self._vehicle_name}/reset_odom"

        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self._reset_odom_publisher = rospy.Publisher(self.reset_odom_topic, Odometry, queue_size=1)
        # construct subscriber
        self.object_topic = f"/{self._vehicle_name}/obstacle_detected"
        self.duck_topic = f"/{self._vehicle_name}/duck_detected"
        self.apriltag_topic = f"/{self._vehicle_name}/apriltag_detector_node/detections"
        self._subscriber_object = rospy.Subscriber(self.object_topic, Bool, self.listen)
        self._subscriber_duck = rospy.Subscriber(self.duck_topic, Bool, self.listen2)
        self._subscriber_tag = rospy.Subscriber(self.apriltag_topic, AprilTagDetectionArray  , self.listen3)
        self.object_detected = False
        self.duck_detected = False
        self.tag_info = [False, 0, 0] # (, side of the robot , front of the robot)
        self.phase_start_time = rospy.get_time()

        self.x, self.y, self.z = 0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0


    

    def sgn(self, x):
        return (x>0)-(x<0)
    

    def turn_right(self, omg= OMEGA):
        message_angle = Twist2DStamped(v=0, omega=omg)
        self._publisher.publish(message_angle)
        return
    def turn_left(self, omg = OMEGA):
        message_angle = Twist2DStamped(v=0, omega=omg*-1)
        self._publisher.publish(message_angle)
        return
    def drive_forward(self, vel=VELOCITY):
        message_angle = Twist2DStamped(v=vel, omega=0)
        self._publisher.publish(message_angle)
        return
    def drive_backward(self, vel=VELOCITY*-1):
        message_angle = Twist2DStamped(v=vel, omega=0)
        self._publisher.publish(message_angle)
        return
    def stop(self):
        message_angle = Twist2DStamped(v=0, omega=0)
        self._publisher.publish(message_angle)
        return
    def detect_obstacle(self):
        return self.object_detected or self.tag_info[0]


    
    def random_walk(self, rate):
        hoek = -1 if randint(0,1) == 0 else 1 #Go left or go right

        tijd =  randint(4,9) # Randomize angular v elocity between -OMEGA and OMEGA rad/s
        for i in range(tijd):
            if(self.detect_obstacle()):
                self.stop()         
                break
            if hoek ==-1:
                self.turn_left()
            else:
                self.turn_right()
            rate.sleep()
                    
        tijd =  randint(5,10)
        for count in range(tijd):
            if(self.detect_obstacle()):
                self.stop()         
                break
            self.drive_forward()
            rate.sleep()
        return
    
    def goto_apriltag(self, info):  
        front = info[2]
        side = info[1]

        if(front>0.8):
            rospy.loginfo("front")
            self.drive_forward()
        if(side>0.15):
            rospy.loginfo("Turn left")
            self.turn_left()
        elif(side<-0.15):
            rospy.loginfo("Turn right")
            self.turn_right()
        elif(front>0.1):
            rospy.loginfo("front")
            self.drive_forward()
        else:
            rospy.loginfo("Draaaiiiii")
            self.turn_left()


    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            time = rospy.get_time() #Return the time in seconds
            self.reset_odometry()
            if(self.tag_info[0]):
                self.goto_apriltag(self.tag_info)
                self.phase_start_time = time
            elif(self.object_detected):
                self.drive_backward()
            
            elif(time-self.phase_start_time>2):
                rospy.loginfo("Start random walk")
                self.random_walk(rate)
            else:
                self.stop()         
            rate.sleep()
             
                
    def reset_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()  # Ideally, should be encoder time
        odom.header.frame_id = self.origin_frame
        odom.pose.pose = Pose(Point(self.x, self.y, self.z), Quaternion(*self.q))
        odom.child_frame_id = self.target_frame
        odom.twist.twist = Twist(Vector3(self.tv, 0.0, 0.0), Vector3(0.0, 0.0, self.rv))

        self._reset_odom_publisher.publish(odom)

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)
        rospy.loginfo("node stopped")
    
    def listen(self, data):
        self.object_detected = data.data
        #rospy.loginfo("Object detected: {%s}", data.data)
    def listen2(self, data):
        self.duck_detected = data.data
        if(self.duck_detected):
            rospy.loginfo("Duck is detected!!!")
    def listen3(self, data : AprilTagDetectionArray):
        count = 0
        for detection in data.detections:
            count+=1
        
        if(count>0):
            rospy.loginfo(f"{count} tags detected!! side: {data.detections[0].transform.translation.x}, front:{data.detections[0].transform.translation.z}")
            self.tag_info = [True,data.detections[0].transform.translation.x, data.detections[0].transform.translation.z]
        else:
            self.tag_info[0] = False
        





if __name__ == '__main__':
    # create the node
    node = SearchApriltagNode(node_name='search_april_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
