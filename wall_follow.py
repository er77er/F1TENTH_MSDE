#!/usr/bin/env python

from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

scan_msg = LaserScan()

#PID CONTROL PARAMS
kp = 5.0 #TODO
kd = 0.09 #TODO
ki = 0.0 #TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)   
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)   

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.s
        #TODO: implement
        idx = int(angle - data.angle_min / data.angle_increment)
        dist = data.ranges[idx]
        if math.isinf(dist) or math.isnan(dist):
            return 4.0

        return dist          

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0
        #TODO: Use kp, ki & kd to implement a PID controller for 
        control_error = kp * error + kd * (error - prev_error)
        angle = angle - math.radians(control_error)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

        print("steering_angle : {}\n".format(angle))
        prev_error = control_error

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        theta = 120
        dist_A = self.getRange(data,theta)
        dist_B = self.getRange(data,180) 
        alpha = math.atan((dist_A * math.cos(theta) - dist_B) / dist_A * math.sin(theta))
        curr_dist = dist_B * math.cos(alpha)
        future_dist = curr_dist + CAR_LENGTH * math.sin(alpha)
        error = leftDist - future_dist
        print("theta : {}".format(theta))
        print("range_120 : {}".format(dist_A))
        print("range_180 : {}".format(dist_B))
        print("alpha : {}".format(alpha))
        print("error : {}".format(error))
        return error 

    def lidar_callback(self, scan_msg):
        """
        """
        global DESIRED_DISTANCE_LEFT

        error = self.followLeft(scan_msg,DESIRED_DISTANCE_LEFT) 
        #send error to pid_control
        if abs(error) < 0.5:
            VELOCITY = 2
        elif abs(error) < 0.7:
            VELOCITY = 1.2
        else:
            VELOCITY = 0.8
        self.pid_control(error, VELOCITY)

def main():
    rospy.init_node("wall_follow", anonymous=True)
    wf = WallFollow()
    wf.lidar_sub
    wf.drive_pub
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main()