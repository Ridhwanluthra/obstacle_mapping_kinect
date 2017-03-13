#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension

def callback(data):
	rospy.loginfo(type(data.data))

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