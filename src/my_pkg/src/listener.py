#! /usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node("reciever")

def event(msg : Int32):
	rospy.loginfo(f"this is the recieved message {msg.data}")


sub = rospy.Subscriber("counter", Int32, event)
rospy.spin()
