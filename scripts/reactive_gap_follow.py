#!/usr/bin/python3
import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.lidar_sub = rospy.Subscriber(
            lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=100)

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # set Field of View
        self.a_idx = int((math.radians(80) - angle_min) / angle_increment)
        self.b_idx = int((math.radians(-80) - angle_min) / angle_increment)

        proc_ranges = list(ranges)

        for i in range(len(proc_ranges)):
            if proc_ranges[i] > 3:
                proc_ranges[i] = 0

        return proc_ranges

    def safety_bubble(self, safety_bubble_radius, ranges, min_distance_idx):
        # Eliminate all points inside 'bubble' (set them to zero)
        while safety_bubble_radius > -safety_bubble_radius:
            ranges[int(min_distance_idx - (safety_bubble_radius))] = 0
            safety_bubble_radius -= 2
        return ranges

    def find_min_distance(self, ranges):
        return min([i for i in ranges[self.b_idx:self.a_idx] if i != 0])

    def find_max_gap(self, free_space_ranges):
        # Find max length gap
        temp = []
        max_gap = []
        max_gap_len = -np.inf
        for i in range(self.b_idx, self.a_idx):
            if free_space_ranges[i] > 2:
                temp.append(free_space_ranges[i])
                if (len(max_gap) > max_gap_len):
                    max_gap = temp
                    max_gap_len = len(max_gap)
                else:
                    temp = []
        print(f"max gap is {max_gap} - max_gap is {len(max_gap)}")
        return max_gap

    def find_best_point(self, max_gap):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
            Naive: Choose the furthest point within ranges and go there
        """
        return (max_gap[int(len(max_gap) / 2)])

    def publish_drive_message(self, angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(
            data.ranges, data.angle_min, data.angle_increment)
        min_distance = self.find_min_distance(proc_ranges)
        proc_ranges = self.safety_bubble(
            safety_bubble_radius=10, ranges=proc_ranges, min_distance_idx=proc_ranges.index(min_distance))
        max_gap = self.find_max_gap(proc_ranges)
        best_point = self.find_best_point(max_gap)
        angle = (data.angle_min +
                 (proc_ranges.index(best_point) * data.angle_increment))


        if abs(angle) > 2:
            speed = 0.5
        elif 1 < abs(angle) <= 2:
            speed = 1.0
        else:
            speed = 1.5
        self.publish_drive_message(angle, speed)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
