#!/usr/bin/env python3
'''
Assume that you have some process that generates noisy data, and hence the resulting information may not be trustworthy. 
An example of that is a sensor measuring distance and returning the distance and the probability that it is correct.
This script simulates such code.
The results are published to a topic called /data with a custom message (dist_prob). The message contains two float64 values: distance and probability.
'''

import rospy
import numpy as np
from data_publisher.msg import dist_prob


def node():
	# define the ROS node called "data_publisher"
	rospy.init_node('data_publisher')  
	
	# create a publisher. Publishes "dist_prob" message to the "data" topic
	pub = rospy.Publisher('data', dist_prob, queue_size=1)
	
	# set the rate of publishing (Hz)
	rate = rospy.Rate(10)
	
	print('Node started. Publishing starting now.')
	print('Use `rostopic echo /data` to view the data.')
	while not rospy.is_shutdown():
		data = dist_prob(np.random.normal(loc=1, scale=0.1), 
						 min(max(0, np.random.normal(loc=0.87, scale=0.1)),1))
		
		# publish the message
		pub.publish(data)
		
		# sleep to maintain the desired publishing rate
		rate.sleep()
		

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass 

