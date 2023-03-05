#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32

rospy.init_node("sender")
# init the note with name "sender"

pub = rospy.Publisher("counter", Int32, queue_size = 10)
# create a publisher object


counter = 0
while not rospy.is_shutdown():
	# as long as the node is not off
	# publish the message
	message = Int32(counter)
	pub.publish(message)
	
	
	rospy.loginfo(message.data)
	# print in the console the message 
	
	counter +=1 
	rospy.sleep(10)
	# increament the counter and wait for 10 seconds
