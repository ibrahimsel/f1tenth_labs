#!/usr/bin/python3
import rospy
import math
import sys
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


# PID CONTROL PARAMS
kp = 0
kd = 0
ki = 0
servo_offset = 0.0
prev_time = 0
prev_error = 0.0
error = 0.0
integral = 0.0


class WallFollow:
    def __init__(self) -> None:
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=10)
        self.speed_x = 0
        self.speed_y = 0
        self.steering_angle = 0

        # TODO: Calculate L  (L is the estimated distance to the left wall after 1 seconds)
        self.L = 2.0

        # a is the laser beam after some theta angle (0 < theta <= 70 )
        self.a = 0
        self.b = 0  # b is the laser beam that shows us the left of the car
        self.Dt = 0  # Distance to left wall
        self.Dt_plus1 = 0  # Estimated distance to left wall after 1 seconds

    def lidar_callback(self, data):
        self.a = data.ranges[int(
            (math.radians(45) - data.angle_min) / data.angle_increment)]
        self.b = data.ranges[int(
            (math.radians(90) - data.angle_min) / data.angle_increment)]
        theta = 45
        
        # right lasers
        # self.c = data.ranges[int(
        #     (math.radians(-45) - data.angle_min) / data.angle_increment)]
        # self.d = data.ranges[int(
        #     (math.radians(-90) - data.angle_min) / data.angle_increment)]
        alpha = np.arctan(
            ((self.a * np.cos(theta)) - self.b) / (self.a * np.sin(theta)))
        self.Dt = self.b * np.cos(alpha)
        self.Dt_plus1 = self.Dt + self.L * np.sin(alpha)

        error = self.calculate_error(0.9, self.Dt_plus1)
        self.pid_control(error)

    def calculate_error(self, desired_distance, current_distance):
        return desired_distance - current_distance

    def pid_control(self, error):
        global integral
        global prev_error
        global prev_time
        global kp
        global ki
        global kd
        kp = 1.0
        ki = 0.001
        kd = 0.005
        angle = 0.0

        current_time = time.time()
        delta_time = prev_time - current_time
        prev_time = current_time
        integral += prev_error * delta_time
        angle = -((kp * error) + (ki * integral) +
                  (kd * ((error - prev_error)) / delta_time))
        prev_error = error

        if abs(angle) > 2:
            velocity = 0.5
        elif 1 < abs(angle) <= 2:
            velocity = 0.7
        else:
            velocity = 1.0

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
