#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests
import math

def callback(data):
	rospy.loginfo(data.data[0])
	min_dist = data.data[1:4]
	angle_right = data.data[4:7]
	angle_left = data.data[7:10]

	rospy.loginfo(min_dist)
	rospy.loginfo(angle_left)
	rospy.loginfo(angle_right)

	width = (angle_right[0] * math.cos(abs((math.pi/4) - angle_right[1]))) + (angle_left[0] * math.cos(abs((math.pi/4) - angle_left[1])))

	r = requests.post("http://www.lithics.in/apis/send_firebase_message.php", data={'message':width})

def listener():
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("cluster_distances", Float64MultiArray, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
