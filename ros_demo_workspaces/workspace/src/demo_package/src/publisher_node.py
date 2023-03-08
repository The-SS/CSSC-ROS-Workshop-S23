#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def node():
	# define the ROS node called "publisher_node"
	rospy.init_node('publisher_node')  
	
	# create a publisher. Publishes "String" message to the "counter" topic
	pub = rospy.Publisher('counter', String, queue_size=1)
	
	# set the rate of publishing: 10Hz
	rate = rospy.Rate(10)
	
	counter = 0
	while not rospy.is_shutdown():
		message_info = "I have counted up to: " + str(counter)
		counter += 1
		
		# publish the message
		pub.publish(message_info)
		print('I published a message with counter = ', counter)
		
		# sleep to maintain the desired publishing rate
		rate.sleep()
		

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass 

