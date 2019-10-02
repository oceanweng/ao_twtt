#!/usr/bin/env python



import rospy, serial, struct, time, copy
from ao_twtt.msg import PDstatus
from std_msgs.msg import String



class Photodiode(object):

	def __init__(self):
		rospy.init_node("pd_driver")
		self._PORT_NAME = rospy.get_param("~PORT_NAME", "/dev/ttyACM0")
		self.port = serial.Serial(self._PORT_NAME, 115200, timeout=0.1)  # default 115200
		self.pub = rospy.Publisher('PD_status', PDstatus, queue_size=10)
		self.msg = PDstatus()






 		time.sleep(1.0)
		self.port.flushInput()  # clear input buffer




	def parse(self,line):
		if len(line) == 0:
			print 'No data'
			return
		elif len(line)!= 7:
			#print line
			return
		elif line[0]!='$':
			print 'ERR0R: Wrong header'
			print line
			return
		time_interval = ord(line[4])
		intensity_H = ord(line[1])
		intensity_L = ord(line[2])
		intensity_H = intensity_H << 8
		intensity = intensity_H + intensity_L	
		self.msg.header.stamp = rospy.get_rostime()
		self.msg.intensity = intensity
		self.pub.publish(self.msg)
		#print intensity

		

	def run(self):
		self.port.flushInput()  # clear input buffer
		self.port.flushOutput()  # clear input buffer
		while not rospy.is_shutdown():
                        #print self.port.in_waiting, self.port.out_waiting
			buf = self.port.readline()
			self.parse(buf)
                        #self.port.flushInput()  # clear input buffer
		        #self.port.flushOutput()  # clear input buffer
                        #time.sleep(0.1)
		#if self.port.readline().startswith('R,'):



if __name__ == "__main__":
	a = Photodiode()
	a.run()
