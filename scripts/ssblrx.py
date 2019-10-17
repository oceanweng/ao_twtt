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
        self._port_name = rospy.get_param('~port_name', '/dev/ttyUSB7')
        self._update_rate = rospy.get_param('~rate', 3) #更新レート
        #変数の設定ここまで。set param使わないと引数２番めのデフォルト値が渡る
        self._port = serial.Serial(self._port_name, 115200, timeout=0.2)
        #self._pub = rospy.Publisher('ssbl_status', SSBL_Status, queue_size=self._update_rate)
        #self._ssbl_status = SSBL_Status()
        self._pub = rospy.Publisher('ssbl_rx', SSBLrx, queue_size=self._update_rate)
        self._ssblrx = SSBLrx()
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
        #self._auv_status = AUV_Status()
        #self._auv_sub = rospy.Subscriber('auv_status', AUV_Status, self.auv_handler)

    def refresh(self):   # データを読み取り、更新する
         try:        
            line = self._port.readline()
            print(line)
            self.parse(line)
         except serial.serialutil.SerialException:
            pass

    def parse(self, line):  #メッセージをデコードして値を更新する
        if len(line) < 8:
            #print 'ERROR: Too short'
            #print line
            #print len(line)
            return
        elif line[0] != '$':
            #print 'ERROR: Wrong header'
            #print line
            #print len(line)
            return

        #print line
        #print line[1:-6]
        dat = self.str2bytes(line[1:-6])  # ヘッダとチェックサム以降を除外
        #print dat
        cid = ord(dat[0])
        #print 'length = ' + str(len(dat)) + ' cid = ' + hex(cid)
        ind = 1
        #self._ssbl_status.stat = 3


        if cid == 0x61:  # CID_NAV_QUERY_RESP p.117

            self._ssblrx.time_SSBLrx = time.time()


            #self.remote[0].time = self.rcvtime  #remoteのインデックスはとりあえずゼロのみとする
            #print(line)
            #print(len(line))
            #if len(line) == 139:
            #     self._ssbl_status.stat = 5
            #elif len(line) == 109:
            #     self._ssbl_status.stat = 4
            #print("stat", self._ssbl_status.stat)
            buf = unpack('<BBBBhhhHHh', dat[ind:ind+16])  # ACOFIX_T p.46
            ind += 16
            self._dest_id = buf[0]    #The ID code of the beacon that this data is sent to.
            self._src_id = buf[1]     #The ID code of the beacon that sent the data.
            #self._ssbl_status.dest_id = self._dest_id
            #self._ssbl_status.src_id = self._src_id
            self._flags = buf[2]      # Bit[0] =RANGE_VALID   Bit[1] =USBL_VALID  Bit[2] =POSITION_VALID Bit[3] =POSITION_ENHANCED
            self._msg_type = buf[3]   # specify a type of acoustic message, 0x1 is MSG_OWAYU
            self._local_yaw = buf[4] / 10.0    #The yaw angle (relative to magnetic north) of the local beacon when the fix was computed.
            self._local_pitch = buf[5] / 10.0  #The pitch angle of the local beacon (接收器自身) when the fix was computed.
            self._local_roll = buf[6] / 10.0   #The roll angle of the local beacon when the fix was computed.
            self._local_depth = buf[7] / 10.0  #The reading from the local beacon depth sensor when the fix was calculated.
            self._velocity_of_sound = buf[8] / 10.0   #The velocity of sound value used for the computation of the remote beacon's range.
            self._rssi = buf[9] / 10.0   #The Received Signal Strength Indicator value for the received message
            #print 'RSSI: ' + str(self._rssi)
            if self._flags & 0b0001: # RANGE_VALID
                buf = unpack('<IiH', dat[ind:ind+10])
                ind += 10
                self._range_time = buf[1] / 10000000.0
                self._range_dist = buf[2] / 10.0
                #print 'Range: ' + str(self._range_dist)
            if self._flags & 0b0010:  # USBL_VALID    这一块在实验时会收到
                n = ord(dat[ind])
                ind += 1
                buf = unpack('<' + str(n + 3) + 'h', dat[ind:ind + (n+3)*2])
                ind += (n+3)*2
                self._usbl_ch = n         #The number of USBL receiver channels being used to compute the signal angle. Typically this value is either 3 or 4.
                for k in range(n):
                    self._usbl_rssi[k] = buf[k] / 10.0    #An array of the received signal strengths for each of the USBL channels
                self._usbl_azimuth = buf[k+1] / 10.0      #The incoming signal azimuth angle from 0° to 360°.
                #self._ssbl_status.azimuth = self._usbl_azimuth
                #print("now", time.time())
                #print("now_publish", self._ssbl_status.mission_time_position)
                #self._ssbl_status.mission_time_position = time.time()
                #print("now_publish_after", self._ssbl_status.mission_time_position)
                self._usbl_elevation = buf[k+2] / 10.0    #The incoming signal elevation angle from -90° to +90°.
                self._usbl_fit_error = buf[k+3] / 100.0   #indicates the quality of fit (or confidence) of the signal azimuth and elevation values
                #print('Azimuth: ', self._usbl_azimuth)
            if self._flags & 0b0100:
                # POSITION VALID
                buf = unpack('<hhh', dat[ind:ind+6])
                ind += 6
                self._pos_east = buf[0] / 10.0
                self._pos_north = buf[1] / 10.0
                self._pos_depth = buf[2] / 10.0
                self._pos_depth2 == self._pos_depth
                temp = self._range_dist**2 - self._pos_north**2 - self._pos_east**2
                if temp > 0:
                    self._pos_depth2 = math.sqrt(temp)
                dir = math.atan2(self._pos_north, self._pos_east) * 180.0 / 3.1415
                hrange = math.sqrt(self._pos_north**2 + self._pos_east**2)
                #print 'N, E, D: ' + str(self._pos_north) + ', ' + str(self._pos_east) + ', ' + ('%.6f' % self._pos_depth)
                #print 'dir, hrange:' + str(int(dir)) + ', ' + str(int(hrange))
                #print 'SSBL_heading: ' + str(int(self._yaw))
                #print 'relative_dir: ' + str(int(dir - self._yaw))
            if self._flags & 0b1000:
                # POSITION ENHANCED
                print 'POSITION ENHANCED'
            if len(line) > 70:
                #self._ssbl_status.mission_time_position = time.time()
                buf = unpack('<BB', dat[ind:ind+2])
                ind += 2
                if buf[1] == 5:   #如果只有五个数据
                    buf = unpack('<ccccc', dat[ind:ind+5])
                    #self._ssbl_status.buf = dat[ind:ind+5]
                    ind += 5
                    buf2 = buf[0]+buf[1]+buf[2]+buf[3]+buf[4]
                    print(buf2)
                    if(buf2 == 'error'):
                        print('surface command')
                if buf[1] == 31:   #如果是31个数据
                    buf = unpack('>HHHBBHBBHHHHHHHBHH', dat[ind:ind+31])
                    self._rxd = unpack('>BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB', dat[ind:ind+31])
                    #print("stat:",self._ssbl_status.stat)
                    #print(len(dat[ind:ind+31]))
                    #self._ssbl_status.buf = dat[ind:ind+31]
                    #self._al_slant = buf[16] * 0.1
                    #self._alrcv_dir = buf[17] * 0.01
                    #print("dist: ", self._al_slant)
                    #print("receive_dir: ", self._alrcv_dir)
                    #print('azimuth: ', self._usbl_azimuth)
                    #print('dest_id:', self._dest_id)
                    #print('src_id:', self._src_id)
                    ind += 31
                    #print(buf[0]/100.0)
        #else:
            #print(line)
            #print len(line)
        self.publish()

    def publish(self):
        # publish time
        self._ssblrx.header.stamp = rospy.Time.now()
        self._pub.publish(self._ssblrx)
        self._port.flushInput()
        self._port.flushOutput()


    def run(self):
        while not rospy.is_shutdown():
			#self.FindCallRcv()
            self.refresh()

            self._rate.sleep()

if __name__ == '__main__':
    rospy.init_node('seatrac')
    seatrac = Seatrac()
    seatrac.run()