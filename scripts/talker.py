#!/usr/bin/env python
# license removed for brevity
'''
*
* project name: 	visual perception for visually impaired	
* author list: 
* filename: 		talker.py
* functions: 		talker
* global variables: NIL
*
'''
import rospy
from std_msgs.msg import String

'''
*
* Function Name: 	talker
* Input: 		NIL
* Output: 		
* Logic: 	Initialises a publisher and publishes hello_str	
* Example Call:	 talker()
*
'''
def talker():
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
