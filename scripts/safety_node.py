#!/usr/bin/python3
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        """
        self.speed_x = 0

        rospy.init_node("safety_node")
        self.pub_ackermann = rospy.Publisher("brake", AckermannDriveStamped, queue_size=100)
        self.pub_brake_bool = rospy.Publisher("brake_bool", Bool, queue_size=100)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)


    def odom_callback(self, odom_msg):
        self.speed_x = odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):
        angle_min = scan_msg.angle_min
        # rospy.loginfo(angle_max)
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        # TTC_safe_threshold = 0.4
        # TTC_mid_threshold = 0.3
        TTC_dangerous_threshold = 0.2 
        min_TTC = 1

        ranges_length = len(ranges)
        for i in range(ranges_length):
            if (ranges[i] != np.NaN) and (ranges[i] != np.inf):
                distance = ranges[i]
            else:
                continue
            angle = angle_min + (angle_increment * i)
            distance_derivative = (np.cos(angle) * self.speed_x)

            if (distance_derivative > 0) and ((distance/distance_derivative) < min_TTC):
                min_TTC = distance / distance_derivative
        
        # rospy.loginfo(f"Min TTC: {min_TTC:.2f}")
        if min_TTC < TTC_dangerous_threshold:
            ack_msg = AckermannDriveStamped()
            brake_msg = Bool()
            ack_msg.drive.speed = 0.0
            brake_msg.data = True
            self.pub_ackermann.publish(ack_msg)
            self.pub_brake_bool.publish(brake_msg)
            
        else:
            self.pub_brake_bool.publish(False)

        # TODO: publish brake message and publish controller bool


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()