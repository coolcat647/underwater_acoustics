#!/usr/bin/env python 
import numpy as np 
import math 
from sympy import *
import time
import rospy 
from scipy import signal as sg

from std_msgs.msg import Int32MultiArray, Float32
from robotx_msgs.msg import HydrophoneData
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import boatStatus
from toolbox_pkg import tool_box_tdoa as tool

class TDOA(object):
    def __init__(self):
        self.ticks = 0.5
        self.bits = 16.0
        self.fs = 192000
        self.c = 1500.0
        self.boat_mic = 1.0
        self.window_time = 1.5
        self.window = self.fs*self.window_time
        self.px = 0
        self.py = -70
        self.heading = 0
        self.index_tdoa = 0
        # filter params
        self.filterIndex = False
        self.filterLowPassCutoff = 40500
        self.filterHighPassCutoff = 9500
        self.filterOrder = 5
        self.twoMicIndex = False
        self.whistle_index = False
        self.highCutoff = 40000
        self.lowCutoff = 25000
        self.overlap_percent = 0.9
        self.SNR_threshold = 20
        self.spect_window = "hann"
	self.threshold = 15
        self.whistle_duration_threshold = 0.4

        self.waitForWindowFullStart = time.time()

        self.boat = boatStatus.boatStatus(self.px, self.py, self.heading, self.fs)
        #On Start up 
        self.OnStartUp()
        self.boat.mic_update(self.boat_mic)
        self.N = int(round(float(self.fs/45.875)))
        self.overlap = self.N* self.overlap_percent
	self.too = tool()

        #print param info
        self.printInitParamsInfo()

        #Iterate Function
        rospy.Timer(rospy.Duration(self.ticks), self.Iterate)
        
        #Publisher
        #self.pub_xxx = rospy.Publisher("Angle", Float64, queue_size=1)
        self.pub_whistleData    = rospy.Publisher("whistle_data", numpy_msg(Floats), queue_size=1)
        self.pub_whistleT       = rospy.Publisher("whistle_t", numpy_msg(Floats), queue_size=1)
        self.pub_whistleF       = rospy.Publisher("whistle_f", numpy_msg(Floats), queue_size=1)

        self.pub_realDataSave1   = rospy.Publisher("real_data_c1", numpy_msg(Floats), queue_size=1)
        self.pub_realDataSave2   = rospy.Publisher("real_data_c2", numpy_msg(Floats), queue_size=1)
        self.pub_realDataAngle  = rospy.Publisher("real_angle", Float32, queue_size=1)
        self.pub_realDataFs     = rospy.Publisher("real_fs", Float32, queue_size=1)

        #Subscriber
        self.sub_data = rospy.Subscriber("hydrophone_data", HydrophoneData, self.receiveMicData, queue_size=1)
        self.sub_heading = rospy.Subscriber("heading", Int32MultiArray, self.upDateHeading, queue_size=1)
        self.sub_x = rospy.Subscriber("x", Int32MultiArray, self.upDateX, queue_size=1)
        self.sub_y = rospy.Subscriber("y", Int32MultiArray, self.upDateY, queue_size=1)


    def OnStartUp(self):
        if rospy.has_param('~appTicks'):
            self.ticks = rospy.get_param('~appTicks')
        if rospy.has_param('~bits'):
            self.bits = rospy.get_param('~bits')
        if rospy.has_param('~fs'):
            self.fs = rospy.get_param('~fs')
        if rospy.has_param('~soundSpeed'):
            self.c = rospy.get_param('~soundSpeed')
        if rospy.has_param('~lengthOfMic'):
            self.boat_mic = rospy.get_param('~lengthOfMic')/(3**0.5)
        if rospy.has_param('~x'):
            self.boat.x = rospy.get_param('~x')
        if rospy.has_param('~y'):
            self.boat.y = rospy.get_param('~y')
        if rospy.has_param('~heading'):
            self.boat.h = rospy.get_param('~heading')
        if rospy.has_param('~sen'):
            self.boat.sen = rospy.get_param('~sen')
        if rospy.has_param('~snr_threshold'):
            self.threshold = rospy.get_param('~snr_threshold')
        if rospy.has_param("~highcutoff"):
            self.highCutoff = rospy.get_param("~highcutoff")
        if rospy.has_param("~lowcutoff"):
            self.lowCutoff = rospy.get_param("~lowcutoff")
        if rospy.has_param("~use2Mic"):
            if(rospy.get_param('~use2Mic') == "true"):
                self.twoMicIndex = True
        #spect_params
        if rospy.has_param("~window_type"):
            self.spect_window = rospy.get_param("~window_type")
        if rospy.has_param("~overlap"):
            self.overlap_percent = rospy.get_param("~overlap")
        if rospy.has_param("~duration_threshold"):
            self.whistle_duration_threshold = rospy.get_param("~duration_threshold")

    def printInitParamsInfo(self):
        print("twoMic               : ", self.twoMicIndex)
        print("fs                   : ", self.fs)
        print("bits                 : ", self.bits)
        print("boatToMic            : ", self.boat_mic)
        print("Sensitivity          : ", self.boat.sen)
        print("SNR_threshold        : ", self.threshold)
        print("highCutoff           : ", self.highCutoff)
        print("lowCutoff            : ", self.lowCutoff)
        print("spect_overlap        : ", self.overlap_percent)
        print("spect_window         : ", self.spect_window)
        print("spect_N              : ", self.N)
        print("duration_threshold   : ", self.whistle_duration_threshold)

    def Iterate(self, event):
        self.waitForWindowFullEnd = time.time()
        if (self.waitForWindowFullEnd - self.waitForWindowFullStart) >= self.window_time:
            #self.apub_xxx.publish(data)
            time_start_cal = time.time()
            self.index_tdoa = 0

            m1_tmp = self.boat.m1.receiveData[:int(self.fs)].copy()
            f, t, whistle_data = self.too.whistle_detector(m1_tmp, self.fs, self.spect_window, self.N, self.overlap, self.threshold, self.lowCutoff, self.highCutoff)
            self.whistle_index, self.duration = self.too.exist_whistle_or_not(whistle_data, self.whistle_duration_threshold, t[1]-t[0])

            if (self.twoMicIndex == False):
                if(self.whistle_index):
                    self.boat.m1.rt = 0
                    m1_data = self.boat.m1.receiveData.copy()
                    m2_data = self.boat.m2.receiveData.copy()
                    m3_data = self.boat.m3.receiveData.copy()
                    self.boat.m2.rt = self.corrXT(m1_data, m2_data, self.boat.m1_m2_dl)
                    self.boat.m3.rt = self.corrXT(m1_data, m3_data, self.boat.m1_m2_dl)
                    self.tdoa(self.boat.m1.x, self.boat.m1.y, self.boat.m1.rt, self.boat.m2.x, self.boat.m2.y, self.boat.m2.rt, self.boat.m3.x, self.boat.m3.y, self.boat.m3.rt)
                    self.waitForWindowFullStart = time.time()
            elif self.twoMicIndex == True:
                if(self.whistle_index):
                    self.boat.m1.rt = 0
                    m1_data = self.boat.m1.receiveData.copy()
                    m2_data = self.boat.m2.receiveData.copy()
                    self.boat.m2.rt = self.corrXT(m1_data, m2_data, self.boat.m1_m2_dl)
                    self.boat.guessAngle = self.Mic2Angle(self.boat.m1.rt, self.boat.m2.rt)
                    self.index_tdoa = 2
      #              self.pubSaveData(m1_data, m2_data, self.fs, self.boat.guessAngle)
                    self.waitForWindowFullStart = time.time()

            #pub whistle data to tdoa_plot.py to plot
            self.pubPlotData(whistle_data, f, t)

            self.printInfo(self.index_tdoa)
            
            time_end_cal = time.time()
            print("Time use = ", time_end_cal-time_start_cal)
            print("------------------------------------------------")

        self.boat.mic_update(self.boat_mic)

    def pubPlotData(self, input_data, f, t):
        self.pub_whistleData.publish(input_data.astype(np.float32).ravel())
        self.pub_whistleF.publish(f.astype(np.float32))
        self.pub_whistleT.publish(t.astype(np.float32))

    def pubSaveData(self, input_data_c1, input_data_c2, fs, angle):
        self.pub_realDataSaveC1.publish(input_data_c1.astype(np.float32))
        self.pub_realDataSaveC2.publish(input_data_c2.astype(np.float32))
        self.pub_realDataFs.publish(fs)
        self.pub_realDataAngle.publish(angle)

    # Cross Correction to find the time difference
    def corrXT(self, a_data, b_data, r):
        a = a_data.copy()
        b = b_data.copy()
        la = len(a)
        lb = len(b)
        d = int(math.ceil(r/self.c*self.fs*1.1))
        tmp = np.zeros(d*2-1)
        for i in range(d):
            tmp[d+i-1] = np.sum(a[i:]*b[:lb-i])
            tmp[d-i-1] = np.sum(b[i:]*a[:la-i])
        maxArgu = np.argmax(tmp)
        real_recevie_t = round((maxArgu-float(d)+1)/self.fs, 16)
        #real_recevie_t = (maxArgu-float(d)+1)/self.fs
        print("real_recevie_t = ", real_recevie_t)
        return real_recevie_t

    # find the angle with three mic
    def tdoa(self, x1, y1, t1, x2, y2, t2, x3, y3, t3):
        x, y = symbols('x, y')
        angle_guess = 0
        eq1 = ((x-x1)**2.0+(y-y1)**2.0)**0.5-((x-x3)**2.0+(y-y3)**2.0)**0.5-self.c*(t3-t1)
        eq2 = ((x-x1)**2.0+(y-y1)**2.0)**0.5-((x-x2)**2.0+(y-y2)**2.0)**0.5-self.c*(t2-t1)
        eq3 = ((x-x2)**2.0+(y-y2)**2.0)**0.5-((x-x3)**2.0+(y-y3)**2.0)**0.5-self.c*(t3-t2)
        pos = solve([eq1, eq2, eq3], [x, y])
        if pos:
            print("Roots = ", pos)
            if pos[0][0].is_Float: 
                if len(pos) > 1:
                    first_dis = math.sqrt((self.boat.x-pos[0][0])**2+(self.boat.y-pos[0][1])**2)
                    second_dis = math.sqrt((self.boat.x-pos[1][0])**2+(self.boat.y-pos[1][1])**2)
                    indexBigger = np.argmax([first_dis, second_dis])
                    if indexBigger == 1:
                        angle_guess = self.angleP2P(pos[1])
                        self.boat.guessAngle = angle_guess
                    else:
                        angle_guess = self.angleP2P(pos[0])
                        self.boat.guessAngle = angle_guess
                else:
                    angle_guess = self.angleP2P(pos[0])
                    self.boat.guessAngle = angle_guess

                self.index_tdoa = 1
            else:
                #rospy.loginfo("pos is complex")
                self.index_tdoa = 2
        else:
            #rospy.loginfo("pos is empty.")
            self.index_tdoa = 2

        if self.index_tdoa == 2:
            self.boat.guessAngle = self.Mic2Angle(t1, t2)

    # find the angle with two mic
    def Mic2Angle(self, t1, t2):
        dt = t2-t1
        l = self.boat.m1_m2_dl
        if (self.c*dt/l > 1 or self.c*dt/l < -1):
            return 400
        theta = 90-math.acos(self.c*dt/l)*180/math.pi
        theta = theta+self.boat.h*180/math.pi
        if theta < 0:
            theta = theta+360
        return round(theta, 1)

    def angleP2P(self, pos):
        x, y = pos
        a = 90-math.degrees(math.atan2(y-self.boat.y, x-self.boat.x))
        if a < 0:
            a = a+360
        return round(a, 1)

    def printInfo(self, index):
        print("Get whistle or not : ", self.whistle_index)
        print("Duration = ", self.duration)
        if index == 1:
            print("Cal by TDOA...")
            rospy.loginfo("Angle is [%f]." %(self.boat.guessAngle))
        elif index == 2:
            print("Cal by two mic...")
            rospy.loginfo("Angle is [%f]." %(self.boat.guessAngle))


    def receiveMicData(self, msg):
        self.boat.m1.receiveData = self.dataUpdate(self.boat.m1.receiveData, self.changeToVolt(msg.data_ch1))
        self.boat.m2.receiveData = self.dataUpdate(self.boat.m2.receiveData, self.changeToVolt(msg.data_ch2))
        if self.twoMicIndex == False:
            self.boat.m3.receiveData = self.dataUpdate(self.boat.m3.receiveData, self.changeToVolt(msg.data_ch3))
        rospy.loginfo("Get Hydrophone data")

    def dataUpdate(self, old_data, new_data):
        #B is new data and A need to be refreshed.
        a = old_data.copy()
        b = new_data.copy()
        c = np.hstack((a[len(b):], b))
        return c

    def changeToVolt(self, data):
        tmp = np.asarray(data).copy().astype(np.float64)
        tmp[tmp > 0] = tmp[tmp > 0]/(2.0**(self.bits-1.0)-1.0)
        tmp[tmp < 0] = tmp[tmp < 0]/(2.0**self.bits-1.0)
        return tmp

    def upDateHeading(self, msg):
        self.boat.h = msg.data
        
    def upDateX(self, msg):
        self.boat.x = msg.data

    def upDateY(self, msg):
        self.boat.y = msg.data

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('tdoa', anonymous=False)
    tdoa = TDOA()
    rospy.on_shutdown(tdoa.onShutdown)
    rospy.spin()
