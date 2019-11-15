#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
seatrac: control BluePrint seatrac sensor
publisher: SSBL_Status
sublisher: SSBL_Command
'''

import rospy
#from captain.msg import AUV_Status
#from dpt6000.msg import DepthStatus
#from seatrac.msg import SSBL_Command
#from seatrac.msg import SSBL_Status
from time import sleep
import serial, time, math
import numpy as np
from struct import *
from ao_twtt.msg import SSBLrx

class Seatrac(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._update_rate = rospy.get_param('~rate', 3) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        #self._pub = rospy.Publisher('ssbl_status', SSBL_Status, queue_size=self._update_rate)
        #self._ssbl_status = SSBL_Status()
        self._pub = rospy.Publisher('ssbl_rx', SSBLrx, queue_size=self._update_rate)
        self._ssblrx = SSBLrx()
        self._rate = rospy.Rate(self._update_rate)

        self._dest_id = rospy.get_param('~dest_id', 0) # 音響信号の宛先ID (0 - 15, 0:broadcast)
        self._src_id = rospy.get_param('~src_id', 0) # 音響信号の送信元ID (1-15)
        self._flags = 0  # uint8 see p.46 
        self._msg_type = 0
        self._local_roll = 0.0  # Roll angle [deg]
        self._local_pitch = 0.0  # Pitch angle [deg]
        self._local_yaw = 0.0  # Yaw angle [deg] rel. magnetic north
        self._roll = 0.0  # Roll angle [deg]
        self._pitch = 0.0  # Pitch angle [deg]
        self._yaw = 0.0  # Yaw angle [deg] rel. magnetic north
        self._local_depth = 0.0  # [m]
        self._depth = 0.0  # [m]
        self._velocity_of_sound = 0.0  # Velocity of Sound [m/s]
        self._rssi = 0.0  # Received Signal Strength Indicator [dB]
        self._range_time = 0.0  # [sec]
        self._range_dist = 0.0  # [m]
        self._usbl_ch = 0  # usually 3 or 4
        self._usbl_rssi = [0.0, 0.0, 0.0, 0.0]  # [dB]
        self._usbl_azimuth = 0.0  # [deg] (0 - 360)
        self._usbl_elevation = 0.0  # [deg] (-90 - 90)
        self._usbl_fit_error = 0.0  # quality of fit (0 is good, 2 or 3 is not good) 
        self._pos_east = 0.0  # Easting distance [m]
        self._pos_north = 0.0  # Northing [m]
        self._pos_depth = 0.0  # Depth from the surface [m]
        self._pos_depth2 = 0.0  # Depth estimated from range, north and east [m]
        self._id = 1  # ID code of the beacon  (1 to 15)
        self._time = 0  # last updated time (UNIX time)
        self._local_clock = 0  # beacon's clock [sec]
        self._voltage = 0.0  # Supply voltage [V]
        self._temperature = 0.0  # Temperature [degC]
        self._pressure = 0.0  # External pressure [MPa]  originally, [Bar]
        self._t_ping = 0
        self._t_log = 0
        self._t_start = 0
        self._ping_interval = 10.0
        self._txd = [0] * 31
        self._rxd = [0] * 31
        self._start_flag = 0
        #self._auv_status = AUV_Status()
        #self._auv_sub = rospy.Subscriber('auv_status', AUV_Status, self.auv_handler)


    def parse(self):  #メッセージをデコードして値を更新する


        self._ssblrx.time_SSBLrx = time.time()
        self._ssblrx.velocityofsound = 1000
        self.publish()

    def publish(self):
        # publish time
        self._ssblrx.header.stamp = rospy.Time.now()
        self._pub.publish(self._ssblrx)



    def run(self):
        while not rospy.is_shutdown():
			#self.FindCallRcv()
            self.parse()
            time.sleep(10.0)

if __name__ == '__main__':
    rospy.init_node('seatrac')
    seatrac = Seatrac()
    seatrac.run()