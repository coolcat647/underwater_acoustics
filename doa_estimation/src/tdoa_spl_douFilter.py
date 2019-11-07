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
from task2.srv import *

CONFIG_ROSSERVICE_VERS = False

class TDOA_spl(object):
    def __init__(self):
        self.ticks = 0.5
        self.bits = 32.0
        self.fs = 192000
        self.c = 1500.0
        self.boat_mic = 1.0
        self.window_time = 0.5
        self.window = self.fs*self.window_time
        self.index_tdoa = 0
        self.lowPass = 41000
        self.highPass = 1000
        self.m1_m2_dl = 1.632
        self.sen = -208
        self.angle_buffer = np.array([])
        self.angle_buffer_len = 11
        self.thresholdTimeofMean = 10
        self.smallThreshold = 130
        self.startIndex = False
        self.getDataIndex = False
        self.corrFlag = False
        self.SaveMsg = RealTimeData()
        self.doorWidth = 15 
        self.doorDistance = 5

        self.waitForWindowFullStart = time.time()

        #On Start up 
        self.OnStartUp()
        self.tool = tool_box_tdoa()

        #print param info
        self.printInitParamsInfo()
        self.m1 = boatStatus.micPosition(int(self.window))
        self.m2 = boatStatus.micPosition(int(self.window))

        #Subscriber
        self.sub_data = rospy.Subscriber("hydrophone_data", HydrophoneData, self.receiveMicData, queue_size=1)
        self.sub_start_index = rospy.Subscriber("index", Bool, self.callIndex, queue_size=1)

        #Iterate Function
        rospy.Timer(rospy.Duration(self.ticks), self.Iterate)
        
        if CONFIG_ROSSERVICE_VERS:
            #ROS Service
            # Note: 'AngleEstimation' is created automatically by catkin_make to make '.srv file' be a 'Object'.
            # This file would be found in devel/lib/python2.7/dist-packages/task2/srv/_AngleEstimation.py
            # Just remember that it has the same name as .srv filename.
            self.srv = rospy.Service('tdoa_spl_douFilter', AngleEstimation, self.serviceCallback)
            self.angle = 0.0
        else:
            #Publisher
            self.pub_angle          = rospy.Publisher("guess_angle", Float32, queue_size=1)
            self.pub_real_data = rospy.Publisher("acoustic_real_data", RealTimeData, queue_size=1)
            self.startIndex = True



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
        if rospy.has_param('~doorWidth'):
            self.doorWidth = rospy.get_param('~doorWidth')
        if rospy.has_param('~doorDistance'):
            self.doorDistance = rospy.get_param('~doorDistance')
        if rospy.has_param('~smallThreshold'):
            self.smallThreshold = rospy.get_param('~smallThreshold')
        if rospy.has_param('~corrFlag'):
            self.corrFlag = rospy.get_param('~corrFlag')

    def printInitParamsInfo(self):
        print("App Ticks            : ", self.ticks)
        print("fs                   : ", self.fs)
        print("bits                 : ", self.bits)
        print("Sensitivity          : ", self.sen)
        print("Low Pass             : ", self.lowPass)
        print("High Pass            : ", self.highPass)
        print("Threshold of peak    : ", self.thresholdTimeofMean)
        print("First Threshold      : ", self.smallThreshold)
        print("-------------------------------------")

    def serviceCallback(self, request):
        self.startIndex = True
        while(1):
            if self.angle_buffer.size >= self.angle_buffer_len:
                # 'AngleEstimationResponse()' is created by catkin_make the the same as 'AngleEstimation' object
                resp = AngleEstimationResponse()
                resp.angle = np.median(self.angle_buffer)
                resp.process_state = 0;
                self.angle_buffer = np.array([])
                break
        self.startIndex = False
        return resp


    def Iterate(self, event):
        self.waitForWindowFullEnd = time.time()
        if (self.waitForWindowFullEnd - self.waitForWindowFullStart) >= self.window_time and self.startIndex == True and self.getDataIndex == True:
            self.maxAngle = math.atan(self.doorWidth/2/self.doorDistance)*180/math.pi+10
            print("Angle boundary = ", self.maxAngle)

            time_start_cal = time.time()
            ch1prefilter = self.tool.butter_bandpass_filter(self.m1.receiveData, self.fs, self.highPass, self.lowPass)
            ch2prefilter = self.tool.butter_bandpass_filter(self.m2.receiveData, self.fs, self.highPass, self.lowPass)
            if(20*math.log10(np.amax(ch1prefilter))-self.sen >= self.smallThreshold and 20*math.log10(np.amax(ch2prefilter))-self.sen >= self.smallThreshold 
                    and np.mean(ch1prefilter)*self.thresholdTimeofMean <= np.amax(ch1prefilter) and np.mean(ch2prefilter)*self.thresholdTimeofMean <= np.amax(ch2prefilter)):
 #           if(np.mean(ch1prefilter)*self.thresholdTimeofMean <= np.amax(ch1prefilter) and np.mean(ch2prefilter)*self.thresholdTimeofMean <= np.amax(ch2prefilter)):
                f1, Pxx1 = sg.welch(ch1prefilter, self.fs, nperseg=1024)
                centerFreq = f1[np.argmax(Pxx1)]
                print("Centre Freq = ", centerFreq)

                ch1filter = self.tool.butter_bandpass_filter(self.m1.receiveData, self.fs, centerFreq-250, centerFreq+250)
                ch2filter = self.tool.butter_bandpass_filter(self.m2.receiveData, self.fs, centerFreq-250, centerFreq+250)
                c1 = ch1filter 
                c2 = ch2filter
                ch1filter = ch1filter**2
                ch2filter = ch2filter**2
                c1Threshold = np.mean(ch1filter)*self.thresholdTimeofMean
                c2Threshold = np.mean(ch2filter)*self.thresholdTimeofMean
                c1BiggerIndex, _ = find_peaks(ch1filter, height=c1Threshold, distance=10)
                c2BiggerIndex, _ = find_peaks(ch2filter, height=c2Threshold, distance=10)

                print("C1 Threshold SPL = ", 10*math.log10(c1Threshold)-self.sen)
                print("C2 Threshold SPL = ", 10*math.log10(c2Threshold)-self.sen)
                print("C1 Max SPL       = ", 10*math.log10(np.amax(ch1filter))-self.sen)
                print("C2 Max SPL       = ", 10*math.log10(np.amax(ch2filter))-self.sen)

                if c1BiggerIndex.size != 0 and c2BiggerIndex.size != 0:
                    print("Over Threshold!")
                    self.guessAngle = self.findDiffT(c1, c2, self.m1_m2_dl/self.c)
                    if self.guessAngle == 400:
                        print("Get error time difference.")
                    elif self.guessAngle == 500:
                        print("Receive echo.")
                    elif self.guessAngle <= self.maxAngle and self.guessAngle >= -self.maxAngle:
                        self.angle_buffer = np.append(self.angle_buffer, self.guessAngle)
                        rospy.loginfo("Pub real angle = %f" %(self.guessAngle))
                        if CONFIG_ROSSERVICE_VERS:
                            pass 
                        else:
                            self.pubSaveData(self.m1.receiveData, self.m1.receiveData, self.fs, self.guessAngle)
                            self.pub_angle.publish(self.guessAngle)
                        # print("guess angle   = ", self.angle_buffer)
            time_end_cal = time.time()
            print("Time use = ", time_end_cal-time_start_cal)
            print("------------------------------------------------")
            self.getDataIndex = False

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
        tmp[tmp < 0] = tmp[tmp < 0]/(2.0**(self.bits-1.0))
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

    def findDiffT(self, inputData1, inputData2, maxDt):
        abs_inputData1 = np.absolute(inputData1)
        abs_inputData2 = np.absolute(inputData2)
        count = 2.0
        while(1):
            BiggerIndex1, _ = find_peaks(abs_inputData1, height=np.mean(abs_inputData1)*count, distance=20)
            BiggerIndex2, _ = find_peaks(abs_inputData2, height=np.mean(abs_inputData2)*count, distance=20)
            if BiggerIndex1.size != 0 and BiggerIndex2.size != 0:
                t1 = BiggerIndex1[0]/self.fs
                t2 = BiggerIndex2[0]/self.fs
                if t1 <= 0.002 or t2 <= 0.002:
                    return 500
                dt = t1-t2
                if(dt > -maxDt and dt < maxDt):
                    if(self.corrFlag == False):
                        index, angle = self.tool.twoMicsAngle(self.m1_m2_dl, t1, t2, self.c)
                        print("C1 receive time  = ", t1)
                        print("C2 receive time  = ", t2)
                        return -angle
                    elif(self.corrFlag == True):
                        return findDiffWithCorr(inputData1, inputData2)
                if count > 10:
                    return 400
                count = count+0.5
            else: return 400

    def findDiffWithCorr(self, inputData1, inputData2):
        t = self.tool.corrXT(inputData1, inputData2, self.m1_m2_dl, self.c, self.fs)
        index, angle = self.tool.twoMicsAngle(self.m1_m2_dl, 0, t, self.c)
        print("Time Difference = ", t)
        return -angle

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('tdoa_spl_douFilter', anonymous=False)
    tdoa_spl = TDOA_spl()
    rospy.on_shutdown(tdoa_spl.onShutdown)
    rospy.spin()
