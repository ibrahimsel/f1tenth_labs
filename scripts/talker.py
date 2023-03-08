#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Float32
from sensor_msgs.msg import LaserScan

pub_max = rospy.Publisher('farthest_point', Float32, queue_size=10)
pub_min = rospy.Publisher('closest_point', Float32, queue_size=10)

def talker():
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

def callback(data):
    data_list = data.ranges
    min_range = min(data_list)
    max_range = max(data_list)
    pub_max.publish(min_range)
    pub_min.publish(max_range)
    # rospy.loginfo(f"max range is: {max_range}, min range is: {min_range}")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

