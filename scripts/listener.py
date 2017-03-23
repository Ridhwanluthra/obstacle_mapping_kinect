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
prev_frame = 0

sentence = ""
width = list()
angle = list()
direction = list()
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
    global curr_frame, data_per_frame, sentence, width, angle, direction, val
    # rospy.loginfo("Data.data")
    # rospy.loginfo(data.data)

    # rospy.loginfo("Data[0]")
    # rospy.loginfo(data.data[0])

    # unpacking the data
    min_dist = data.data[1:4]
    angle_right = data.data[4:7]
    angle_left = data.data[7:10]

    if curr_frame == data.data[0]:
        data_per_frame.append(data.data)
    else:
        for dat in data_per_frame:
            min_dist = dat[1:4]
            angle_right = dat[4:7]
            angle_left = dat[7:10]
            # calculates the width of the object using the angle to each of the extreme sides and their distances
            width.append((angle_right[0] * math.cos(abs((math.pi/4) - angle_right[1]))) + (angle_left[0] * math.cos(abs((math.pi/4) - angle_left[1]))))
            angle.append((math.pi/4) - min_dist[1])
            direction.append("right" if angle[-1] < 0 else "left")
        sentence = "sss"
        for i in range(len(width)):
<<<<<<< HEAD
=======
            sentence += "there is a " + str(int(width[i])) + " meter wide object towards your " + direction[i] + " at an angle of " + str(abs(angle[i] * (180 / math.pi))) + " and "
        sentence = sentence[:-6]
        r = requests.get("http://www.lithics.in/apis/eyic/getStatus.php")

        rospy.loginfo('r.content')
        rospy.loginfo(r.content)
        rospy.loginfo('val')
        rospy.loginfo(val)
        
        if val != int(r.content):
            print(sentence)
            rp = requests.post("http://www.lithics.in/apis/eyic/firebase.php", data={'message':sentence})
        
        val = int(r.content)
        
        # sentence = ""
>>>>>>> 6e650cc5d311c61ae3196d54ecb876ed5e38aa00
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
