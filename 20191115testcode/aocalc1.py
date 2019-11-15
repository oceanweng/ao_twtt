#!/usr/bin/env python



import rospy, serial, struct, time, copy
from ao_twtt.msg import PDstatus, SSBLrx, AOcalc
from std_msgs.msg import String



class AcousticOptical(object):

	def __init__(self):
		rospy.init_node("ao_calculation")
		self.pub = rospy.Publisher('ao_calc', AOcalc, queue_size=10)
		self.msg = AOcalc()
		rospy.Subscriber('PD_status', PDstatus, self._update_pd_time)
		rospy.Subscriber('ssbl_rx', SSBLrx, self._update_ssbl_time)
		self.ssbl_time = 0
		self.pd_time = 0
		self.timediff = 0
		self.velocityofsound = 0

	def _update_ssbl_time(self, msg):
                #print("update ssbl")
		self.ssbl_time = msg.header.stamp
		#print(self.ssbl_time)
		#self.ssbl_time = msg.time_SSBLrx
		self.velocityofsound = msg.velocityofsound

	def _update_pd_time(self, msg):	
		#print("update pd time")
                t = rospy.Time(10)
		self.pd_time = msg.header.stamp-t
		print(self.pd_time)
		print(self.ssbl_time)
		if self.ssbl_time != 0 :	
			self.timediff = self.ssbl_time - self.pd_time
			print(self.timediff)
			if self.timediff < rospy.Time(10) :
				seconds = self.timediff.to_sec()
				#nanoseconds = self.timediff.to_nsec()
				#print(seconds)
				#print(nanoseconds)
				self.range = self.velocityofsound * seconds
				print(self.range)
				self.msg.header.stamp = rospy.get_rostime()
				self.msg.inter_range = self.range
				self.pub.publish(self.msg)





if __name__ == "__main__":
	a = AcousticOptical()
	rospy.spin()
