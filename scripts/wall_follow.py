#!/usr/bin/python3
# from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0 # TODO arrange control params
kd = 0 # TODO arrange control params
ki = 0 # TODO arrange control params
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=100)
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.speed_x = 0
        self.speed_y = 0
        self.steering_angle = 0

        # TODO: Calculate L  (L is the estimated distance to the left wall after 1 seconds)
        self.L = 0.5

        # a is the laser beam after some theta angle (0 < theta <= 70 )
        self.a = 0  
        self.b = 0  # b is the laser beam that shows us the left of the car
        self.Dt = 0  # Distance to left wall
        self.Dt_plus1 = 0  # Estimated distance to left wall after 1 seconds


    def getRange(self, data, theta):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        return 0.0

    def pid_control(self, error, velocity):
        # global integral
        # global prev_error
        # global kp
        # global ki
        # global kd
        # angle = 0.0
        # # TODO: Use kp, ki & kd to implement a PID controller for 
        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "laser"
        # drive_msg.drive.steering_angle = angle
        # drive_msg.drive.speed = velocity
        # self.drive_pub.publish(drive_msg)
        pass

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return 0.0 

    def lidar_callback(self, data):
        self.a = data.ranges[int((math.radians(150) - data.angle_min) / data.angle_increment)]
        self.b = data.ranges[int((math.radians(90) - data.angle_min) / data.angle_increment)]
        theta = 60 * data.angle_increment
        alpha = np.arctan(
            ((self.a * np.cos(theta)) - self.b) / (self.a * np.sin(theta)))
        self.Dt = self.b * np.cos(alpha)
        self.Dt_plus1 = self.Dt + self.L * np.sin(alpha)
        print(f"Distance to the left wall in a time is: {self.Dt_plus1}")

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = 1
        drive_msg.drive.speed = 2
        self.drive_pub.publish(drive_msg)
        error = 0.0  # TODO: replace with error returned by followLeft
        # send error to pid_control
        self.pid_control(error, VELOCITY)
    
    def odom_callback(self, odom_msg):
        # self.speed_x = odom_msg.twist.twist.linear.x
        # self.speed_y = odom_msg.twist.twist.linear.y
        pass

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
