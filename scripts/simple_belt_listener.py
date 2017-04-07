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
##import gpio_control as gc
import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests
import math

right_pin = 8
left_pin = 7
center_pin = 6

pins = [left_pin, center_pin, right_pin]
directions = ['left', 'center', 'right']

sentence = ""

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
    global sentence, val
    try:
        for i in range(3):
            if data.data[i] < 1:
                print(directions[i])
                gc.switch_on(pins[i])
            else:
                # pass
                gc.switch_off(pins[i])
            sentence += directions[i] + ": " +str(round(data.data[i], 2)) + ", "
        sentence = sentence[:-2]
        print(sentence)

        r = requests.get("http://www.lithics.in/apis/eyic/getStatus.php")
        if val != int(r.content):
            rp = requests.post("http://www.lithics.in/apis/eyic/firebase.php", data={'message':sentence})
        val = int(r.content)
        sentence = ""

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
    rospy.Subscriber("simple_distances", Float64MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    listener()
