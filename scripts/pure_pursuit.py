#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion, PoseStamped
from std_msgs.msg import Header, ColorRGBA, String
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import os
from visualization_msgs.msg import Marker, MarkerArray

LOOKAHEAD_DISTANCE = 0.5

waypoints_file = open(
    (os.path.expanduser('~')+'/rcws/logs/wp-2023-03-13-14-35-37.csv'), 'r')

ct = 0

class PurePursuit(object):
    def __init__(self):
        self.pose_sub = rospy.Subscriber(
            '/gt_pose', PoseStamped, self.pose_callback)
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback)
        self.steering_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=10)
        self.visualize_pub = rospy.Publisher('visualizing', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
        

    def scan_callback(self, scan_msg):
        pass
        
        
    def visualize_waypoints(self):
        pass

    def pose_callback(self, pose_msg):
        global ct
        self.pose = pose_msg
        self.x = pose_msg.pose.position.x
        self.y = pose_msg.pose.position.y
        self.theta = pose_msg.pose.orientation.z
        contents = waypoints_file.read()
        lines = contents.splitlines()
        for i in lines:
            ct += 1
            splitted = i.split(",")
            x = float(splitted[0])
            y = float(splitted[1])
            # angle = 
            # speed = 
            # self.publish_steering_angle(speed, angle)
            # print(f"x: {x}, y: {y}")
            marker = Marker()
            marker.id = ct
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_array.markers.append(marker)
        self.visualize_pub.publish(self.marker_array)

    def calculate_steering_angle(self, lookahead_point_x, lookahead_point_y, x, y):
        dx = lookahead_point_x - x
        dy = lookahead_point_y - y
        return np.arctan2(dy, dx)

    def publish_steering_angle(self, speed, angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "pure_pursuit"
        msg.drive.steering_angle = angle
        msg.drive.speed = speed
        self.steering_pub.publish(msg)


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
    waypoints_file.close()



if __name__ == '__main__':
    main()
