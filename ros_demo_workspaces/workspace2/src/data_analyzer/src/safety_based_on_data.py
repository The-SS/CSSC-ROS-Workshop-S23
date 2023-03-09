#!/usr/bin/env python3
'''
We have some data and a corresponding probability and we want to take a decision on it.
For example, if the distance corresponds to the clearance between a robot and a wall, then we need to decide the robot's actions based on the distance and the probability of it being correct.

For simplicity, we will say that the robot is safe if the distance is higher than some threshold (d_min) and the probability is higher than some other threshold (pmin). 
'''

import rospy
from data_publisher.msg import dist_prob
from std_msgs.msg import String

class DataSubscriber:
	def __init__(self):
		self.data_sub = rospy.Subscriber('data', dist_prob, self.callback, queue_size=1)
		self.data = None
	def callback(self, data):
		self.data = data

class DecisionPublisher:
	def __init__(self):
		self.decision_pub = rospy.Publisher('decision', String, queue_size=1)
		self.dmin = 0.85  # minimum distance
		self.pmin = 0.80 # minimum confidence
	def decide(self, data):
		if data is None:
			self.decision_pub.publish("No data yet >_<   >_<   >_<")
		elif data.distance < self.dmin:
			message = 'Distance less than threshold: ' + str(data.distance) + '<' + str(self.dmin)
			self.decision_pub.publish(message)
		elif data.probability < self.pmin:
			message = 'Probability less than threshold: ' + str(data.probability) + '<' + str(self.pmin)
			self.decision_pub.publish(message)
		else:
			self.decision_pub.publish("Safe!")
		return
		
def main():
	rospy.init_node('data_analyzer')  
	rate = rospy.Rate(10)
	
	sub = DataSubscriber()
	pub = DecisionPublisher()
	
	print('Data analyzer started.')
	print('Use `rostopic echo /decision` to view the decisions.')
	while not rospy.is_shutdown():
		pub.decide(sub.data)
		rate.sleep()
		

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass 

