#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

# TODO: import ROS msg types and libraries
odom_msg = Odometry()
scan_msg = LaserScan()
drive_msg = AckermannDriveStamped()
safety_on = Bool()
ttc_threshold = rospy.get_param("/safety_node/ttc_threshold")

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self,speed):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = odom_msg.twist.twist.linear.x
        #initialize parameters
        drive_msg.drive.speed = 0
        scan_msg.ranges = []

        # TODO: create ROS subscribers and publishers.
        
        

    #stop the car
    def ttc_action(self):
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.steering_angle_velocity = 0
        drive_msg.drive.speed = 0
        drive_msg.drive.acceleration = 0
        drive_msg.drive.jerk = 0
        safety_on.data = True   

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.odom_msg = odom_msg
        drive_msg.drive.speed = odom_msg.twist.twist.linear.x
            
    def scan_callback(self, scan_msg):
        
        #publish False in nornmal driving 
        self.scan_msg = scan_msg
        safety_on.data = False
        drive_msg.drive.speed = odom_msg.twist.twist.linear.x
        
        # TODO: calculate TTC
        if self.speed != 0:
            for i in range(len(scan_msg.ranges)):
                proj_vel = odom_msg.twist.twist.linear.x * math.cos(i/4)
                ttc = scan_msg.ranges[i] / proj_vel
                # TODO: publish brake message and publish controller bool
                if (ttc<=ttc_threshold) and (ttc > 0):
                    self.ttc_action()
                    rospy.loginfo("Emergency brake engaged!!!")
        
    
  
def main():
    rospy.init_node('safety_node',anonymous = False)
    sn = Safety(drive_msg.drive.speed)
    rospy.Subscriber('/odom',Odometry,sn.odom_callback)
    rospy.Subscriber('/scan',LaserScan,sn.scan_callback)
    brake_pub = rospy.Publisher('/brake',AckermannDriveStamped,queue_size=10)
    brake_bool_pub = rospy.Publisher('/brake_bool',Bool,queue_size=10)

    brake_pub.publish(drive_msg)
    brake_bool_pub.publish(safety_on)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInternalException:
        pass
#Recieve scan data from lidar (LaserScan)then, publish message(AckermannCriveStamped) to the car to stop
