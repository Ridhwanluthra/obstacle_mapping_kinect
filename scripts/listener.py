#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests
import math
from time import sleep

curr_frame = int()
data_per_frame = list()
check = True

def callback(data):
	global curr_frame, data_per_frame, check
	rospy.loginfo(data.data[0])

	min_dist = data.data[1:4]
	angle_right = data.data[4:7]
	angle_left = data.data[7:10]

	rospy.loginfo(min_dist)
	rospy.loginfo(angle_left)
	rospy.loginfo(angle_right)
	
	width = list()
	angle = list()

	if curr_frame == data.data[0]:
		data_per_frame.append(data.data)
	else:
		for dat in data_per_frame:
			min_dist = dat[1:4]
			angle_right = dat[4:7]
			angle_left = dat[7:10]
			width.append((angle_right[0] * math.cos(abs((math.pi/4) - angle_right[1]))) + (angle_left[0] * math.cos(abs((math.pi/4) - angle_left[1]))))
			angle.append((math.pi/4) - min_dist[1])
		print(len(width), len(angle), curr_frame, data.data[0])
		direction0 = "right" if angle[0] < 0 else "left"
		direction1 = "right" if angle[1] < 0 else "left"
		sentence = "there is a " + str(int(width[0])) + "meter wide object towards your " + direction0 + "there is a "  + str(int(width[1])) + "meter wide object towards your " + direction1
	curr_frame = data.data[0] + 1
	if check:	
		r = requests.post("http://www.lithics.in/apis/send_firebase_message.php", data={'message':sentence})
		check = False
	check = False
	sleep(30)

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
