#!/usr/bin/env python
"""Plot the live microphone signal(s) with matplotlib.

Matplotlib and NumPy have to be installed.

"""
import argparse
import Queue
import sys
#from __future__ import with_statement
#from __future__ import absolute_import

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np
import sounddevice as sd

import rospy, serial, struct, time, copy
from std_msgs.msg import String
from ao_twtt.msg import ACstatus

class Acousticrx(object):

	def __init__(self):
		rospy.init_node("ac_rx")
		#self._PORT_NAME = rospy.get_param("~PORT_NAME", "/dev/ttyACM0")
		#self.port = serial.Serial(self._PORT_NAME, 115200, timeout=0.1)  # default 115200
		self.pub = rospy.Publisher('AC_status', ACstatus, queue_size=10)
		self.msg = ACstatus()


		time.sleep(1.0)

	def publ(self):
		self.msg.header.stamp = rospy.get_rostime()
		self.pub.publish(self.msg)
		time.sleep(1.0)




def audio_callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print >>sys.stderr, status
    # Fancy indexing with mapping creates a (necessary!) copy:
    #q.put(indata[::args.downsample, mapping])
    print np.max(indata)
    if np.max(indata) > 0.8:
        print 'ok'
        a.publ()




a = Acousticrx()

try:

    fig, ax = plt.subplots()


    stream = sd.InputStream(
        device=5, channels=1,
        samplerate=48000, callback=audio_callback)
    with stream:
        plt.show()
except Exception, e:
    parser.exit(type(e).__name__ + u': ' + unicode(e))

