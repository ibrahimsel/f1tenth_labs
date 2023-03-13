#!/usr/bin/python3
import rospy
import numpy as np
import atexit
import tf
import time
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math


home = expanduser('~')
file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

prev_waypoint = {'x': 0, 'y': 0}

def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])
    

    euler = euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    
    dist = 0.0
    dx = data.pose.pose.position.x - prev_waypoint['x']
    dy = data.pose.pose.position.y - prev_waypoint['y']
    dist = (math.pow(dx, 2) + math.pow(dy, 2))
    if dist > 0.1:
        file.write(f'{data.pose.pose.position.x}, {data.pose.pose.position.y}, {euler[2]}, {speed}\n')
        prev_waypoint['x'] = data.pose.pose.position.x
        prev_waypoint['y'] = data.pose.pose.position.y

    # if data.twist.twist.linear.x > 0.:
        # print(f'x: {data.pose.pose.position.x}, y: {data.pose.pose.position.y}, euler: {euler[2]}, speed: {speed}\n')
    


def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('odom', Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass