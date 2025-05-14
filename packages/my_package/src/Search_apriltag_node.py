#!/usr/bin/env python3

import os
import rospy
import time
from random import uniform
from random import randint
from std_msgs.msg import Bool  # Import Bool message type
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, AprilTagDetectionArray, AprilTagDetection, LEDPattern
from geometry_msgs.msg import Transform, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf import transformations as tr
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform
from duckietown_msgs.srv import SetCustomLEDPattern
import numpy as np
from tf import transformations as tr




# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 1.0     # angular vel   , rad/s     , counter clock wise (+)
TILE = 0.6
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
        self.odom_topic = f"/{self._vehicle_name}/deadreckoning_node/odom"

        self._subscriber_object = rospy.Subscriber(self.object_topic, Bool, self.listen_object)
        self._subscriber_duck = rospy.Subscriber(self.duck_topic, Bool, self.listen_duck)
        self._subscriber_tag = rospy.Subscriber(self.apriltag_topic, AprilTagDetectionArray  , self.listen_tag)
        self._subscriber_odom = rospy.Subscriber(self.odom_topic, Odometry  , self.listen_odom)
        #self._subscriber_odomtry = rospy.Subscriber(self.)

        #Variables
        self.object_detected = False
        self.duck_detected = False
        self.tag_info = [False, 0, 0] # (, side of the robot , front of the robot)
        self.phase_start_time = rospy.get_time()

        # Services
        self.led_service = rospy.ServiceProxy(f"/{self._vehicle_name}/led_emitter_node/set_custom_pattern", SetCustomLEDPattern)
        self.led_pattern = LEDPattern()
        self.led_pattern.color_list = [0,0,0,0,0]
        self.led_pattern.color_mask = [1, 1, 1, 1, 1]
        self.led_pattern.frequency = 1
        self.led_pattern.frequency_mask = [0,0,0,0,0]


        #Calibration
        self.past_apriltags = None
        self.count_calibration = 0
        self.calibration_time = rospy.get_time()

        #Coordinate variables
        self.x, self.y, self.z = 0 , 0 , 0
        self.yaw = 0.0
        self.q = [0.0, 0.0, 0.0, 1.0]
        self.tv = 0.0
        self.rv = 0.0

        self.x_listen, self.y_listen, self.z_listen, self.yaw_listen = 0, 0, 0, 0
        
        self.apriltag_coord_dict = {
                                    11 : (0.566 + TILE*6, TILE*5 + 0.03),
                                    33 : (0.752 + TILE*6, TILE*5 + 0.03),
                                    303 : (9*TILE+ 0.02 , 2.481),
                                    301 : (9*TILE + 0.02, 2.373),
                                    25 : (9*TILE, 1.573),
                                    32 : (9*TILE, 1.371),
                                    302 : (9*TILE, 0.502),
                                    9 : (9*TILE, 0.380),
                                    304 : ( 9*TILE - 0.434, -0.01),
                                    305 : (9*TILE - 0.561, -0.01),
                                    10 : (7*TILE + 0.02, 0.692),
                                    57 : (7*TILE + 0.02, 0.879),
                                    27 : (6*TILE, 5*TILE - 1.016),
                                    26 : (6*TILE, 5*TILE - 0.810)
                                    }




    

    def sgn(self, x):
        return (x>0)-(x<0)
    
    

    def estimate_robot_pose(self, p1_robot, p2_robot, p1_world, p2_world):
        p1_r = np.array(p1_robot)
        p2_r = np.array(p2_robot)
        p1_w = np.array(p1_world)
        p2_w = np.array(p2_world)

        vec_r = p2_r - p1_r
        vec_w = p2_w - p1_w


        theta_r = np.arctan2(vec_r[1], vec_r[0])
        theta_w = np.arctan2(vec_w[1], vec_w[0])
        #theta in redians
        theta = theta_w - theta_r

        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        t = p1_w - R @ p1_r

        theta = theta + np.math.pi/2
        q = tr.quaternion_from_euler(0,0,theta)
        return t[0] , t[1] , q



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
        message_angle = Twist2DStamped(v=vel, omega=OMEGA*2)
        self._publisher.publish(message_angle)
        return
    def stop(self):
        message_angle = Twist2DStamped(v=0, omega=0)
        self._publisher.publish(message_angle)
        return
    def detect_obstacle(self):
        return self.object_detected or self.duck_detected
    
    
    def turn_towards(self, target_angle):
        while abs(self.yaw - target_angle)>0.1:
            if(self.yaw<target_angle):
                self.turn_right
            else:
                self.turn_left
            rospy.sleep(0.1)
        self.stop()
    
    def goto_coordinate(self, x, y):
        current_x, current_y = self.x, self.y
        distance = np.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
        angle_to_target = np.arctan2(y - current_y, x - current_x)
        self.turn_towards(angle_to_target)
        while distance > 0.1:
            #Miss nog iets van bij object detected brake --> effe random walk en dan opnieuw
            self.drive_forward()
            rospy.sleep(0.1)
            current_x, current_y = self.x, self.y
            distance = np.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
        self.stop()
 


    
    def random_walk(self, rate):
        hoek = -1 if randint(0,1) == 0 else 1 #Go left or go right

        tijd =  randint(4,9) # Randomize angular v elocity between -OMEGA and OMEGA rad/s
        for i in range(tijd):
            if(self.detect_obstacle()):
                rospy.loginfo("Object detectd --> stop and break random walkj")
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
                rospy.loginfo("Object detectd --> stop and break random walkj")
                self.stop()         
                break
            self.drive_forward()
            rate.sleep()
        return
    
    def goto_apriltag(self, info):
        front = info[1][1]
        side = info[1][0]   
        count = info[0]
        if(count>1):
            self.calibrate(info)
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
        # else:
        #     rospy.loginfo("Draaaiiiii")
        #     self.turn_left()

    def calibrate(self, info):
        
        
        self.stop()
        rospy.loginfo(f"trying to calibrate,count = {self.count_calibration}")
        
        id1 = info[1][2]
        id2 = info[2][2]

        if(self.past_apriltags is None or id1 not in self.past_apriltags or id2 not in self.past_apriltags):
            self.past_apriltags = [id1, id2]
            self.count_calibration = 1
            return
        if(self.count_calibration <5):
            self.count_calibration +=1
            return
        else:
            self.past_apriltags = None
            self.count_calibration =  0
            self.calibration_time = rospy.get_time()
            robot_coord1 = info[1][:2]
            robot_coord2 = info[2][:2]
            world_coord1 = self.apriltag_coord_dict.get(id1)
            world_coord2 = self.apriltag_coord_dict.get(id2)

            if (world_coord1 == None or world_coord2 == None):
                rospy.loginfo("error, unknown tag detected")
                return
            
            self.x, self.y, self.q =  self.estimate_robot_pose(robot_coord1, robot_coord2 ,world_coord1, world_coord2)
            self.reset_odometry()

        

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        rospy.loginfo("Start run")
        while not rospy.is_shutdown():
            time = rospy.get_time() #Return the time in seconds
            if(time - self.calibration_time >15 and self.tag_info[0]):
                self.goto_apriltag(self.tag_info)
                
                self.phase_start_time = time
                #Change leds
                self.change_led_pattern(['blue', 'blue', 'blue', 'blue', 'blue'], [1, 1, 1, 1, 1], 1)
            
            elif(self.object_detected):
                self.change_led_pattern(['green', 'red', 'yellow', 'red', 'green'], [0, 1, 0, 1, 0], 2.5)
                self.drive_backward()
            
            elif(time-self.phase_start_time>0.5):
                rospy.loginfo("Start random walk")
                self.change_led_pattern(['yellow', 'yellow', 'yellow', 'yellow', 'yellow'], [0, 0, 0, 0, 0], 1)
                self.random_walk(rate)
            else:
                self.stop()         
            rate.sleep()
             
                
    def reset_odometry(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()  # Ideally, should be encoder time
        #odom.header.frame_id =x, y, theta self.origin_frame
        odom.pose.pose = Pose(Point(self.x, self.y, self.z), Quaternion(*self.q))
        #odom.child_frame_id = self.target_frame
        odom.twist.twist = Twist(Vector3(self.tv, 0.0, 0.0), Vector3(0.0, 0.0, self.rv))
        
        self._reset_odom_publisher.publish(odom)
        rospy.loginfo(f"odometry succesfully reset to: x : {self.x}, y: {self.y}, theta : {self.yaw} ")

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)
        rospy.loginfo("node stopped")

    def change_led_pattern(self, color_list, frequency_mask, freq):

        #Brakke test om te checken of het nodig is om te updaten
        if(color_list == self.led_pattern.color_list and freq == self.led_pattern.frequency):
            return
        
        self.led_pattern.color_list = color_list # Front L, BAck R, ----, Back L, front R
        self.led_pattern.color_mask = [1, 1, 1, 1, 1]
        self.led_pattern.frequency = freq
        self.led_pattern.frequency_mask = frequency_mask

        
        try:
            self.led_service(self.led_pattern)
            rospy.loginfo("Custom LED pattern set successfully.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    def listen_object(self, data):
        self.object_detected = data.data
        #rospy.loginfo("Object detected: {%s}", data.data)
    def listen_duck(self, data):
        self.duck_detected = data.data
        if(self.duck_detected):
            rospy.loginfo("Duck is detected!!!")
    def listen_tag(self, data : AprilTagDetectionArray):
        count = 0
        for detection in data.detections:
            count+=1
        
        self.tag_info = [count]

        for detection in data.detections:
            id = detection.tag_id
            self.tag_info.append((detection.transform.translation.x, detection.transform.translation.z, id))
    def listen_odometry(self, data):
        # Update position from odometry
        self.x_listen = data.pose.pose.position.x
        self.y_listen = data.pose.pose.position.y
        self.z_listen = data.pose.pose.position.z

        # Extract quaternion from odometry and convert to yaw
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, self.yaw_listen = tr.euler_from_quaternion(q) 


        





if __name__ == '__main__':
    # create the node
    node = SearchApriltagNode(node_name='search_april_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
