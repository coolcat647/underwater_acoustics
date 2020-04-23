#!/usr/bin/env python 
import numpy as np 
import math 
from sympy import *
import time
from scipy import signal as sg
from scipy.signal import find_peaks

import rospy 
from std_msgs.msg import Int32MultiArray, Float32, Bool
from robotx_msgs.msg import HydrophoneData
from robotx_msgs.msg import RealTimeData

import boatStatus
from toolbox_pkg import tool_box_tdoa

class TDOA_spl(object):
    def __init__(self):
        self.ticks = 0.5
        self.bits = 32.0
        self.fs = 192000
        self.c = 1500.0
        self.boat_mic = 1.0
        self.window_time = 0.5
        self.window = self.fs*self.window_time
        self.px = 0
        self.py = 0
        self.heading = 0
        self.index_tdoa = 0
        self.lowPass = 41000
        self.highPass = 1000
        self.m1_m2_dl = 1
        self.sen = -208
        self.angle_buffer = np.array([])
        self.angle_buffer_len = 11
        self.thresholdTimeofMean = 10
        self.startIndex = True
        self.getDataIndex = False
        self.SaveMsg = RealTimeData()

        self.waitForWindowFullStart = time.time()

        #On Start up 
        self.OnStartUp()
	self.tool = tool_box_tdoa()

        #print param info
        self.printInitParamsInfo()
        self.m1 = boatStatus.micPosition(int(self.window))
        self.m2 = boatStatus.micPosition(int(self.window))
        self.m3 = boatStatus.micPosition(int(self.window))

        #Iterate Function
        rospy.Timer(rospy.Duration(self.ticks), self.Iterate)
        
        #Publisher
        self.pub_angle          = rospy.Publisher("guess_angle", Float32, queue_size=1)
        self.pub_real_data = rospy.Publisher("acoustic_real_data", RealTimeData, queue_size=1)

        #Subscriber
        self.sub_data = rospy.Subscriber("hydrophone_data", HydrophoneData, self.receiveMicData, queue_size=1)
        self.sub_start_index = rospy.Subscriber("index", Bool, self.callIndex, queue_size=1)


    def OnStartUp(self):
        if rospy.has_param('~appTicks'):
            self.ticks = rospy.get_param('~appTicks')
        if rospy.has_param('~bits'):
            self.bits = rospy.get_param('~bits')
        if rospy.has_param('~fs'):
            self.fs = rospy.get_param('~fs')
        if rospy.has_param('~soundSpeed'):
            self.c = rospy.get_param('~soundSpeed')
        if rospy.has_param('~micToMic'):
            self.m1_m2_dl = rospy.get_param('~micToMic')
        if rospy.has_param('~sen'):
            self.sen = rospy.get_param('~sen')
        if rospy.has_param('~lowPass'):
            self.lowPass = rospy.get_param('~lowPass')
        if rospy.has_param('~highPass'):
            self.highPass = rospy.get_param('~highPass')
        if rospy.has_param('~windowLen'):
            self.window = rospy.get_param('~windowLen')
        if rospy.has_param('~angleLen'):
            self.angle_buffer_len = rospy.get_param('~angleLen')
        if rospy.has_param('~meanTime'):
            self.thresholdTimeofMean = rospy.get_param('~meanTime')

    def printInitParamsInfo(self):
        print("App Ticks            : ", self.ticks)
        print("fs                   : ", self.fs)
        print("bits                 : ", self.bits)
        print("Sensitivity          : ", self.sen)
        print("Low Pass             : ", self.lowPass)
        print("High Pass            : ", self.highPass)
        print("Threshold of peak    : ", self.thresholdTimeofMean)
        print("-------------------------------------")

    def Iterate(self, event):
        self.waitForWindowFullEnd = time.time()
        if (self.waitForWindowFullEnd - self.waitForWindowFullStart) >= self.window_time and self.startIndex == True and self.getDataIndex == True:
            #self.apub_xxx.publish(data)
            time_start_cal = time.time()

            ch1prefilter = self.tool.butter_bandpass_filter(self.m1.receiveData, self.fs, self.highPass, self.lowPass)
            f1, Pxx1 = sg.welch(ch1prefilter, self.fs, nperseg=self.fs)
            centerFreq = f1[np.argmax(Pxx1)]
            print("Centre Freq = ", centerFreq)

            ch1filter = self.tool.butter_bandpass_filter(self.m1.receiveData, self.fs, centerFreq-500, centerFreq+500)
            ch2filter = self.tool.butter_bandpass_filter(self.m2.receiveData, self.fs, centerFreq-500, centerFreq+500)
            ch3filter = self.tool.butter_bandpass_filter(self.m3.receiveData, self.fs, centerFreq-500, centerFreq+500)
            ch1filter = ch1filter**2
            ch2filter = ch2filter**2
            ch3filter = ch3filter**2
            c1Threshold = np.mean(ch1filter)*self.thresholdTimeofMean
            c2Threshold = np.mean(ch2filter)*self.thresholdTimeofMean
            c3Threshold = np.mean(ch3filter)*self.thresholdTimeofMean
            c1BiggerIndex, _ = find_peaks(ch1filter, height=c1Threshold, distance=2)
            c2BiggerIndex, _ = find_peaks(ch2filter, height=c2Threshold, distance=2)
            c3BiggerIndex, _ = find_peaks(ch3filter, height=c3Threshold, distance=2)

            print("C1 Threshold SPL = ", 10*math.log10(c1Threshold)-self.sen)
            print("C2 Threshold SPL = ", 10*math.log10(c2Threshold)-self.sen)
            print("C3 Threshold SPL = ", 10*math.log10(c3Threshold)-self.sen)
            print("C1 Max SPL       = ", 10*math.log10(np.amax(ch1filter))-self.sen)
            print("C2 Max SPL       = ", 10*math.log10(np.amax(ch2filter))-self.sen)
            print("C3 Max SPL       = ", 10*math.log10(np.amax(ch3filter))-self.sen)

            if c1BiggerIndex.size != 0 and c2BiggerIndex.size != 0 and c3BiggerIndex.size != 0:
                self.m1.rt = float(c1BiggerIndex[0])/self.fs
                self.m2.rt = float(c2BiggerIndex[0])/self.fs
                self.m3.rt = float(c3BiggerIndex[0])/self.fs

                index1, self.guessAngle1 = self.tool.twoMicsAngle(self.m1_m2_dl, self.m1.rt, self.m2.rt, self.c)
                index2, self.guessAngle2 = self.tool.twoMicsAngle(self.m1_m2_dl, self.m1.rt, self.m3.rt, self.c)
                index3, self.guessAngle3 = self.tool.twoMicsAngle(self.m1_m2_dl, self.m3.rt, self.m2.rt, self.c)
                #if self.guessAngle != 400:
                print("C1 receive time  = ", self.m1.rt)
                print("C2 receive time  = ", self.m2.rt)
                print("C3 receive time  = ", self.m3.rt)
                print("guess angle1   = ", self.guessAngle1)
                print("guess angle2   = ", self.guessAngle2+60)
                print("guess angle3   = ", self.guessAngle3-60)
                self.angle_buffer = np.append(self.angle_buffer, self.guessAngle)
                self.pubSaveData(self.m1.receiveData, self.m1.receiveData, self.fs, self.guessAngle)
            
            time_end_cal = time.time()
            print("size of buffer = ", self.angle_buffer.size)
            if self.angle_buffer.size >= self.angle_buffer_len:
                angle = np.median(self.angle_buffer)
                self.angle_buffer = np.array([])
                self.startIndex = False
                self.pub_angle.publish(angle)
                rospy.loginfo("Pub real angle = %f" %(angle))
            print("Time use = ", time_end_cal-time_start_cal)
            print("------------------------------------------------")
            self.getDataIndex = False

#    def changeToSpl(self, volt, sen):
        

    def receiveMicData(self, msg):
        self.m1.receiveData = self.dataUpdate(self.m1.receiveData, self.changeToVolt(msg.data_ch1))
        self.m2.receiveData = self.dataUpdate(self.m2.receiveData, self.changeToVolt(msg.data_ch2))
        rospy.loginfo("Get Hydrophone data")
        self.getDataIndex = True

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

    def callIndex(self, msg):
        if msg == True:
            self.startIndex = True 

    def pubSaveData(self, input_data_c1, input_data_c2, fs, angle):
        self.SaveMsg.data_ch1 = input_data_c1.tolist()
        self.SaveMsg.data_ch2 = input_data_c2.tolist()
        self.SaveMsg.fs = fs 
        self.SaveMsg.angle = angle
        self.pub_real_data.publish(self.SaveMsg)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('tdoa_spl_douFilter', anonymous=False)
    tdoa_spl = TDOA_spl()
    rospy.on_shutdown(tdoa_spl.onShutdown)
    rospy.spin()
