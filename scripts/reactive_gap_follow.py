#!/usr/bin/python3
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

min_distance = np.inf
min_distance_idx = 0
gap_counter = 0
max_gap = -np.inf
max_gap_last_idx = 0
max_gap_first_idx = 0
best_point_idx = 0

class reactive_follow_gap:
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=100)

    def preprocess_lidar(self, ranges, angle_min, angle_increment):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.a_idx = int((math.radians(30) - angle_min) / angle_increment)
        self.b_idx = int((math.radians(-30) - angle_min) / angle_increment)

        proc_ranges = list(ranges)

        for i in range(len(proc_ranges)):
            if proc_ranges[i] > 3:
                proc_ranges[i] = 0

     
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        return None

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        global min_distance, min_distance_idx, max_gap, max_gap_last_idx, gap_counter, max_gap_first_idx, best_point_idx
        proc_ranges = self.preprocess_lidar(data.ranges, data.angle_min, data.angle_increment)

        for i in range(self.b_idx, self.a_idx):
            if data.ranges[i] < min_distance:
                min_distance = data.ranges[i]
                min_distance_idx = i
                continue

        # print(min_distance)

        # Eliminate all points inside 'bubble' (set them to zero) 
        safety_bubble_radius = 10
        while safety_bubble_radius > -10:
            proc_ranges[min_distance_idx - int(safety_bubble_radius / 2)] = 0
            safety_bubble_radius -= 2
        
        
        # Find max length gap
        for i in range(self.b_idx, self.a_idx):
            if proc_ranges[i] != 0:
                gap_counter += 1
                if gap_counter > max_gap:
                    max_gap = gap_counter
                    max_gap_last_idx = i
                    max_gap_first_idx = i - max_gap
            else:
                gap_counter = 0

        # print(max_gap,max_gap_first_idx, max_gap_last_idx)
        # print(proc_ranges)

        # Find the best point in the gap 
        best_point = max(proc_ranges[max_gap_first_idx:max_gap_last_idx])
        best_point_idx = proc_ranges.index(best_point)
        angle = (data.angle_min + (best_point_idx * data.angle_increment))
        print(best_point_idx, best_point, angle)
        speed = 0.5
        min_distance = np.inf

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)