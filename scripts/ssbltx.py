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

class Seatrac(object):
    def __init__(self):
        #変数の設定。ほぼ固定だが、別ロボットへの移植を考慮してset paramで変えられるようにしている
        self._port_name = rospy.get_param('~port_name', '/dev/ttyUSB7')
        self._update_rate = rospy.get_param('~rate', 3) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._port = serial.Serial(self._port_name, 115200, timeout=0.2)
        #self._pub = rospy.Publisher('ssbl_status', SSBL_Status, queue_size=self._update_rate)
        #self._ssbl_status = SSBL_Status()
        self._rate = rospy.Rate(self._update_rate)
        self._port.flushInput()
        self._port.flushOutput()
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
        self._auv_status = AUV_Status()
        self._auv_sub = rospy.Subscriber('auv_status', AUV_Status, self.auv_handler)

    def sendcommand(self, command):  # コマンド送信
        command_str = ''
        for c in command:
            command_str += hex(c)[2:].zfill(2) # 2桁のhex文字列
        crc = self.calcCRC(command)
        command_str += hex(crc)[4:].zfill(2)
        command_str += hex(crc)[2:4].zfill(2)     
        if hasattr(self, '_port'):
            print 'Sending: #' + command_str
            self._port.write('#' + command_str + '\r\n')
            sleep(0.01)
        else:
            print 'SeaTrac: Cannot send command.  Port closed.'

    def SetData(self, txd):
        command = [0x60, 0x00, 0x01, 31, txd[0], txd[1], txd[2], txd[3], txd[4], txd[5], txd[6], txd[7], txd[8], txd[9], txd[10], txd[11], txd[12], txd[13], txd[14], txd[15], txd[16], txd[17], txd[18], txd[19], txd[20], txd[21], txd[22], txd[23], txd[24], txd[25], txd[26], txd[27], txd[28], txd[29], txd[30]]
        #command = [0x60, 0x02, 0x04, 0x05, 0x00, 0x01, 0x10, 0x01, 0x06]
        print("set data", command)
        self.sendcommand(command)

    def calcCRC(self, input_data):  # CRCの計算　CRCV16IBM
    # Serial Command Interface Reference p.27参照
    # imput_dataはuint8のリスト
        poly = 0xA001
        crc = 0
        for c in input_data:
            for i in range(8):
                if (c & 0x01) ^ (crc & 0x01):
                    crc >>= 1
                    crc ^= poly
                else:
                    crc >>= 1
                c >>= 1
        return crc

    def run(self):
        while not rospy.is_shutdown():

            now = time.time()
            if now > self._t_ping + self._ping_interval:  #USBL測位をする
                self._t_ping = now
                self.SetData(self._txd)

            #self._rate.sleep()
            time.sleep(10.0)

if __name__ == '__main__':
    rospy.init_node('seatrac')
    seatrac = Seatrac()
    seatrac.run()