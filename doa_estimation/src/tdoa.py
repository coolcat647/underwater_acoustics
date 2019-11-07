#!/usr/bin/env python 
import numpy as np 
import math 
from sympy import *
import time
import rospy 
from scipy import signal as sg
from std_msgs.msg import Int32MultiArray, Float64
from robotx_msgs.msg import HydrophoneData

import boatStatus

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
        self.filterHighPassCutoff = 1000
        self.filterOrder = 5
        self.twoMicIndex = True

        self.waitForWindowFullStart = time.time()

        self.boat = boatStatus.boatStatus(self.px, self.py, self.heading, self.fs)
        #On Start up 
        self.OnStartUp()
        self.boat.mic_update(self.boat_mic)

        #print param info
        self.printInitParamsInfo()

        #Iterate Function
        rospy.Timer(rospy.Duration(self.ticks), self.Iterate)
        
        #Publisher
        self.pub_xxx = rospy.Publisher("Angle", Float64, queue_size=1)

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
        if rospy.has_param('~maxSel'):
            self.boat.max_sel = rospy.get_param('~maxSel')
        if rospy.has_param('~filter'):
            if(rospy.get_param('~filter') == "true"):
                self.filterIndex = True
        if rospy.has_param("~highpass"):
            self.filterHighPassCutoff = rospy.get_param("~highpass")
        if rospy.has_param("~lowpass"):
            self.filterLowPassCutoff = rospy.get_param("~lowpass")
        if rospy.has_param("~filterorder"):
            self.filterOrder = rospy.get_param("~filterorder")
        if rospy.has_param("~use2Mic"):
            if(rospy.get_param('~use2Mic') == "true"):
                self.twoMicIndex = True

    def printInitParamsInfo(self):
        print("fs          : ", self.fs)
        print("bits        : ", self.bits)
        print("boatToMic   : ", self.boat_mic)
        print("sen         : ", self.boat.sen)
        print("maxSel      : ", self.boat.max_sel)
        print("filter      : ", self.filterIndex)
        print("highPass    : ", self.filterHighPassCutoff)
        print("lowPass     : ", self.filterLowPassCutoff)
        print("filterOrder : ", self.filterOrder)
        print("twoMic      : ", self.twoMicIndex)

    def Iterate(self, event):
        self.waitForWindowFullEnd = time.time()
        if (self.waitForWindowFullEnd - self.waitForWindowFullStart) >= self.window_time:
            #self.apub_xxx.publish(data)
            time_start_cal = time.time()
            self.index_tdoa = 0
            self.boat.m1.sel = self.CalSEL(self.boat.m1.receiveData.copy(), self.boat.sen)
            self.boat.m2.sel = self.CalSEL(self.boat.m2.receiveData.copy(), self.boat.sen)
            self.boat.m3.sel = self.CalSEL(self.boat.m3.receiveData.copy(), self.boat.sen)
            if (self.twoMicIndex == False):
                if(self.boat.m1.sel >= self.boat.max_sel and self.boat.m2.sel >= self.boat.max_sel and self.boat.m3.sel >= self.boat.max_sel):
                    self.boat.m1.rt = 0
                    m1_data = self.boat.m1.receiveData.copy()
                    m2_data = self.boat.m2.receiveData.copy()
                    m3_data = self.boat.m3.receiveData.copy()
                    self.boat.m2.rt = self.corrXT(m1_data, m2_data, self.boat.m1_m2_dl)
                    self.boat.m3.rt = self.corrXT(m1_data, m3_data, self.boat.m1_m2_dl)
                    self.tdoa(self.boat.m1.x, self.boat.m1.y, self.boat.m1.rt, self.boat.m2.x, self.boat.m2.y, self.boat.m2.rt, self.boat.m3.x, self.boat.m3.y, self.boat.m3.rt)
                    self.waitForWindowFullStart = time.time()
            elif self.twoMicIndex == True:
                if(self.boat.m1.sel >= self.boat.max_sel and self.boat.m2.sel >= self.boat.max_sel):
                    self.boat.m1.rt = 0
                    m1_data = self.boat.m1.receiveData.copy()**2
                    m2_data = self.boat.m2.receiveData.copy()**2
                    self.boat.m2.rt = self.corrXT(m1_data, m2_data, self.boat.m1_m2_dl)
                    self.boat.guessAngle = self.Mic2Angle(self.boat.m1.rt, self.boat.m2.rt)
                    self.index_tdoa = 2
                    self.waitForWindowFullStart = time.time()

            self.printInfo(self.index_tdoa)
            time_end_cal = time.time()
            print("Time use = ", time_end_cal-time_start_cal)
            print("------------------------------------------------")

        self.boat.mic_update(self.boat_mic)

    def CalSEL(self, d, sen):
        data = d[:int(self.fs)].copy()
        total = np.sum(data**2)
        total = total/self.fs
        SEL = round(10*np.log10(total))-sen
        return SEL

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
        print("Channel 1 SEL = ", self.boat.m1.sel)
        print("Channel 2 SEL = ", self.boat.m2.sel)
        print("Channel 3 SEL = ", self.boat.m3.sel)
        if index == 1:
            print("SEL Over! ")
            print("Cal by TDOA...")
            rospy.loginfo("Angle is [%f]." %(self.boat.guessAngle))
        elif index == 2:
            print("SEL Over! ")
            print("Cal by two mic...")
            rospy.loginfo("Angle is [%f]." %(self.boat.guessAngle))
        else:
            print("SEL Not Over! ")

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
        if(self.filterIndex == True):
            tmp_filter = self.butter_bandpass_filter(tmp, self.filterHighPassCutoff, self.filterLowPassCutoff, self.fs, self.filterOrder)
            return tmp_filter
        else:
            return tmp

    def butter_bandpass_filter(self, data, highPass, lowPass, fs, order):
        b, a = self.butter_bandpass(highPass, lowPass, fs, order)
        filteredData = sg.filtfilt(b, a, data)
        return filteredData

    def butter_bandpass(self, highPass, lowPass, fs, order):
        nyquestFs = fs/2.0
        highPass = highPass/nyquestFs
        lowPass = lowPass/nyquestFs
        b, a = sg.butter(order, [highPass, lowPass], btype='bandpass', analog=False)
        return b, a

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
