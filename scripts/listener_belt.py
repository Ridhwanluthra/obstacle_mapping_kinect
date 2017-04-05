#!/usr/bin/env python

'''
*
* project name:     visual perception for visually impaired 
* author list:      Pankaj Baranwal, Ridhwan Luthra, Shreyas Sachan, Shashwat Yashaswi
* filename:         listener.py
* functions:        callback, listener
* global variables: curr_frame, data_per_frame, check
*
'''
import gpio_control as gc
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests
import math
from time import sleep

curr_frame = 0
data_per_frame = list()

width = list()
minDistance = list()
min_angle = list()
left_most = list()
right_most = list()
direction = list()
# send_data = bool()
val = int()

right_pin = 8
left_pin = 7
center_pin = 6

'''
*
* Function Name:    callback
* Input:        data -> data about the point cloud
* Output:       
* Logic:        
* Example Call:  callback function, manual calling not required.
*
'''
def callback(data):
    try:
        global val, curr_frame, data_per_frame, sentence, width, minDistance, min_angle, left_most, right_most, direction

        counter = data.data[0]
        current_width = (data.data[1])
        currentMinDistance = (data.data[2])
        current_min_angle = (data.data[3])
        current_left_most = (data.data[4])
        current_right_most = (data.data[5])
        if curr_frame == data.data[0]:
            data_per_frame.append(data.data)
        else:
            for dat in data_per_frame:
                width.append(round(dat[1], 2))
                minDistance.append(round(dat[2], 2))
                min_angle.append(round(dat[3], 2))
                left_most.append(round(dat[4], 2))
                right_most.append(round(dat[5], 2))
                
                direction.append("right" if min_angle[-1] > 0 else "left")
            for i in range(len(width)):
                if width > 0.0:
                    if abs(min_angle[i] * (180 / math.pi)) < 20:
                        print('center')
                        gc.switch_on(center_pin)
                        gc.switch_off(right_pin)
                        gc.switch_off(left_pin)
                    else:
                        if direction[i] == "right":
                            print('right')
                            gc.switch_on(right_pin)
                            gc.switch_off(center_pin)
                            gc.switch_off(left_pin)
                        else:
                            print('left')
                            gc.switch_on(left_pin)
                            gc.switch_off(right_pin)
                            gc.switch_off(center_pin)
            width = list()
            min_angle = list()
            minDistance = list()
            left_most = list()
            right_most = list()
            direction = list()
            curr_frame = data.data[0]
            data_per_frame = list()
    except Exception as e:
        print(e)
        gc.reset()

'''
*
* Function Name:    listener
* Input:        NIL
* Output:       NIL
* Logic:    Initializes node and makes sure that two nodes don't have the same name, 
*           also subscribes to the topic and calls the callback function.
* Example Call:  listener()
*
'''
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

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    listener()
