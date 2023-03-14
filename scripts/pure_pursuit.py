#!/usr/bin/python3
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
import rospy
import numpy as np
import os
import math

LOOKAHEAD_DISTANCE = 1.0
waypoints_file = open(
    (os.path.expanduser('~')+'/rcws/logs/waypoints_file.csv'), 'r')
ct = 0  # for marker ids

class PurePursuit(object):
    def __init__(self):
        global ct
        self.pose_sub = rospy.Subscriber(
            '/gt_pose', PoseStamped, self.pose_callback)
        # self.scan_sub = rospy.Subscriber(
            # '/scan', LaserScan, self.scan_callback)
        self.steering_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=10)
        self.visualize_pub_markerarray = rospy.Publisher('visualizing', MarkerArray, queue_size=10)
        self.visualize_pub_marker = rospy.Publisher('visualizing2', Marker, queue_size=10)

        self.marker_array = MarkerArray()
        self.waypoints = []
        contents = waypoints_file.read()
        lines = contents.splitlines()
        for i in lines:
            splitted = i.split(",")
            x = float(splitted[0])
            y = float(splitted[1])
            self.waypoints.append([y,x])
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
            ct += 1

    def get_closest_waypoint(self, car_point):
        min_distance = np.inf
        closest_point = self.waypoints[0]
        for i in self.waypoints:
            if math.dist(car_point, i) < min_distance:
                closest_point = i
                min_distance = math.dist(car_point, i)
        marker = Marker()
        marker.id = -1
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = closest_point[1]
        marker.pose.position.y = closest_point[0]
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
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.visualize_pub_marker.publish(marker)
        return closest_point
    
    def get_goal_waypoint(self, car_point, closest_waypoint):
        goal_waypoint = []
        furthest_distance = -np.inf
# 
        for i in self.waypoints:
            distance = math.dist(car_point, i)
            if (distance <= LOOKAHEAD_DISTANCE) and (self.waypoints.index(i) > self.waypoints.index(closest_waypoint)):
                if (distance > furthest_distance):
                    furthest_distance = distance
                    goal_waypoint = i
        
        try:
            marker = Marker()
            marker.id = -2
            marker.header.frame_id = "map"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = goal_waypoint[1]
            marker.pose.position.y = goal_waypoint[0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            self.visualize_pub_marker.publish(marker)
            return goal_waypoint
        except IndexError:
            print("err")
    
    def pose_callback(self, pose_msg):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        theta = pose_msg.pose.orientation.z
        closest_waypoint = self.get_closest_waypoint([y, x])
        goal_waypoint = self.get_goal_waypoint([y, x], closest_waypoint)
        angle = self.calculate_steering_angle(LOOKAHEAD_DISTANCE, abs(goal_waypoint[1] - x))
        angle = math.radians(angle)
        print(angle)
        self.visualize_pub_markerarray.publish(self.marker_array)
        self.publish_steering(0.8, angle)
        
    def calculate_steering_angle(self, L, y):
        return ((2*y)) / (math.pow(L, 2))

    def publish_steering(self, speed, angle):
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
