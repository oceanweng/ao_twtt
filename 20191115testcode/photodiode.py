#!/usr/bin/env python



import rospy, serial, struct, time, copy
from ao_twtt.msg import PDstatus
from std_msgs.msg import String



class Photodiode(object):

	def __init__(self):
		rospy.init_node("pd_driver")
		self.pub = rospy.Publisher('PD_status', PDstatus, queue_size=10)
		self.msg = PDstatus()






 		time.sleep(1.0)
		self.port.flushInput()  # clear input buffer




	def parse(self):

			self.msg.header.stamp = rospy.get_rostime()
			self.pub.publish(self.msg)


		

	def run(self):
		while not rospy.is_shutdown():

			self.parse(buf)
			time.sleep(10.0)


if __name__ == "__main__":
	a = Photodiode()
	a.run()
