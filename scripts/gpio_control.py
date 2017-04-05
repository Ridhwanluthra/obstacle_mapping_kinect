
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
# import rospy
import math
from time import sleep
import RPi.GPIO as gpio
import os

#Defining GPIO pin numbers 
#Leftmost motor
pin1 = 6 # This is GPIO6, hence, 5th pin in inner row from ports
#Middle motor
pin2 = 7 # 8th pin in outer row from ports
#Rightmost motor
pin3 = 8 # 9th pin in outer row from ports

# Setting up basic GPIO settings
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(pin1, gpio.OUT)
gpio.setup(pin2, gpio.OUT)
gpio.setup(pin3, gpio.OUT)

curr_frame = 0
data_per_frame = list()

sentence = ""
width = list()
minDistance = list()
min_angle = list()
left_most = list()
right_most = list()
direction = list()
# send_data = bool()
val = int()

def switch_on(pin_number):
    gpio.output(pin_number, 1)
    
def switch_off(pin_number):
    gpio.output(pin_number, 0)
                
def reset():
    global pin1
    global pin2
    global pin3
    gpio.output(pin1, 0)
    gpio.output(pin2, 0)
    gpio.output(pin3, 0)
    sleep(0.1)

'''
*
* Function Name:    callback
* Input:        data -> data about the point cloud
* Output:       
* Logic:        
* Example Call:  callback function, manual calling not required.
*
'''

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    print ("HIGH!")
    switch_on(pin3)
    sleep(2)
    reset()
    print ("HIGH!")
    switch_on(pin2)
    sleep(2)
    reset()
    print ("HIGH!")
    switch_on(pin1)
    sleep(2)
    reset()
