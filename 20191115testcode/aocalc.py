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



	def _update_ssbl_time(self, ssblmsg):
		self._ssbl_time = self.ssblmsg.header.stamp
		#self._ssbl_time = self.ssblmsg.time_SSBLrx
		self.velocityofsound = self.ssblmsg.velocityofsound

	def _update_pd_time(self, pdmsg):
		self._pd_time = self.pdmsg.header.stamp - 5
		self.timediff = self._ssbl_time - self._pd_time
		if self.timediff < 1 :
			self.range = self.velocityofsound * self.timediff
			self.msg.header.stamp = rospy.get_rostime()
			self.msg.inter_range = self.range
			self.pub.publish(self.msg)





if __name__ == "__main__":
	a = AcousticOptical()
	rospy.spin()
