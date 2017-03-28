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
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests
import math
from time import sleep

curr_frame = 0
data_per_frame = list()

sentence = ""
width = list()
minDistance = list()
min_angle = list()
direction = list()
height = list()
# send_data = bool()
val = int()
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
    global val, curr_frame, data_per_frame, sentence, width, minDistance, min_angle, direction, height

    counter = data.data[0]
    current_width = (data.data[1])
    currentMinDistance = (data.data[2])
    current_min_angle = (data.data[3])
    current_left_most = (data.data[4])
    current_right_most = (data.data[5])
    print ("sdsds")

    if curr_frame == data.data[0]:
        data_per_frame.append(data.data)
    else:
        for dat in data_per_frame:
            width.append(round(dat[1], 2))
            minDistance.append(round(dat[2], 2))
            min_angle.append(round(dat[3], 2))
            height.append(round(dat[4], 2))
            
            direction.append("right" if min_angle[-1] > 0 else "left")
        for i in range(len(width)):
        	if width > 0.0:
                    sentence += "there is a " + str(width[i]) + " meter wide object "  + " towards your " + direction[i] + " at an angle of " + str(round(abs(min_angle[i] * (180 / math.pi)), 2)) + " degrees   and "
        if len(sentence)>6:
            print ("222sdsds")
            sentence = sentence[:-6]
            r = requests.get("http://www.lithics.in/apis/eyic/getStatus.php")

            # rospy.loginfo('r.content')
            # rospy.loginfo(r.content)
            # rospy.loginfo('val')
            # rospy.loginfo(val)
            print ("333sdsds")
            print (r.content)
            print (val)
            if val != int(r.content):
                print(sentence)
                rp = requests.post("http://www.lithics.in/apis/eyic/firebase.php", data={'message':sentence})
            
            val = int(r.content)
        
        sentence = ""
        width = list()
        angle = list()
        direction = list()
        curr_frame = data.data[0]
        data_per_frame = list()

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
