#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

rospy.init_node('listener', anonymous=True)

def callback_max(data):
    rospy.loginfo(f"Max range is:{data.data}")

def callback_min(data):
    rospy.loginfo(f"Min range is {data.data}")


def listener():
    rospy.Subscriber('farthest_point', Float32, callback_max)
    rospy.Subscriber('closest_point', Float32, callback_min)
    rospy.spin()


if __name__ == '__main__':
    listener()
