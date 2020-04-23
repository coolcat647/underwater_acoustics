#!/usr/bin/env python 
import numpy as np 
import math 
from sympy import *
import time
import rospy 
from scipy import signal as sg
from robotx_msgs.msg import HydrophoneData
from numpy.fft import irfft, rfft 
from doa_estimation.srv import *
import boatStatus

class TDOA(object):
    def __init__(self):
        self.ticks = 0.5
        self.bits = 32.0
        self.fs = 192000
        self.c = 1500.0
        self.boat_mic = 1.0
        self.window_time = 1.5
        self.window = self.fs*self.window_time
        self.px = 0
        self.py = -70
        self.heading = 0
        # filter params
        self.filterIndex = False
        self.filterLowPassCutoff = 45000
        self.filterHighPassCutoff = 8000
        self.filterOrder = 5
        self.twoMicIndex = True

        self.m1_m2_dl = 1
        self.sen = -208
        self.angle_buffer = np.array([])
        self.angle_buffer_len = 11

        self.waitForWindowFullStart = time.time()

        self.boat = boatStatus.boatStatus(self.px, self.py, self.heading, self.fs)
        
        #On Start up 
        self.OnStartUp()

        #print param info
        self.printInitParamsInfo()
        self.m1 = boatStatus.micPosition(int(self.window))
        self.m2 = boatStatus.micPosition(int(self.window))

        #Iterate Function
        rospy.Timer(rospy.Duration(self.ticks), self.Iterate)

        #Publisher
        self.pub_xxx = rospy.Publisher("Angle", Float64, queue_size=1)

        #Subscriber
        self.sub_data = rospy.Subscriber("hydrophone_data", HydrophoneData, self.receiveMicData, queue_size=1)

        #ROS Service
        # Note: 'AngleEstimation' is created automatically by catkin_make to make '.srv file' be a 'Object'.
        # This file would be found in devel/lib/python2.7/dist-packages/task2/srv/_AngleEstimation.py
        # Just remember that it has the same name as .srv filename.
        self.srv = rospy.Service('tdoa_v2', AngleEstimation, self.serviceCallback)
        self.boat.m1_m2_dl = 1.5
        self.flag_getdata = False
        
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
            self.sen = rospy.get_param('~sen')
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

    # The Service callback function need to define a 'request' param, it represent the 'request from the client'.
    # Response to client by declare 'AngleEstimationResponse' and assign the responsed value as below.
    def serviceCallback(self, request):
        while(True):
            if self.flag_getdata == True:
                self.flag_getdata = False      # clear the flag quickly to avoid data congestion
                time_start_cal = time.time()
                m1_data = self.boat.m1.receiveData.copy()
                m2_data = self.boat.m2.receiveData.copy()
                if self.twoMicIndex == True:
                    m1_data = self.butter_bandpass_filter(m1_data, lowcut=8000, highcut=20000, fs=self.fs, order=5)
                    m2_data = self.butter_bandpass_filter(m2_data, lowcut=8000, highcut=20000, fs=self.fs, order=5)
                    m1_m2_xcorr = self.xcorr(m1_data, m2_data)
                    idx_max = np.argmax(abs(m1_m2_xcorr))
                    ## There are different result of cross-corr between Python and Matlab. The second one
                    ## has a process for 'signal shift', so we do the compensation for Python version.
                    if idx_max < self.fs/2:
                        idx_max = idx_max + self.fs

                    timeDiff = (float(idx_max)- (self.fs))/self.fs
                    #print 'idx_max=', idx_max, ' timeDiff=', timeDiff
                    self.boat.m1.rt = 0
                    self.boat.m2.rt = timeDiff
                    self.boat.m1_m2_dl = 1.1
                    self.boat.guessAngle = self.Mic2Angle(self.boat.m1.rt, self.boat.m2.rt)
                    # If output is not error, add the result to list
                    if(self.boat.guessAngle != 400):
                        self.angle_buffer = np.append(self.angle_buffer, self.boat.guessAngle)
                        rospy.loginfo("Angle is [%f]." %(self.boat.guessAngle))
                    #print("Time use = ", time.time()-time_start_cal)
                    #print("------------------------------------------------")
                    #self.boat.mic_update(self.boat_mic)
                    
                if self.angle_buffer.size >= self.angle_buffer_len:
                    # Declare the service response object
                    resp = AngleEstimationResponse()
                    resp.angle = np.median(self.angle_buffer)
                    self.angle_buffer = np.array([])
                    return resp

    def CalSEL(self, d, sen):
        data = d[:int(self.fs)].copy()
        total = np.sum(data**2)
        total = total/self.fs
        if total != 0:
            SEL = round(10*np.log10(total))-sen
        else:
            SEL = -np.inf
        return SEL

    def xcorr(self, a, b):
        # Do normalization or not
        # nom = np.linalg.norm(a[:])*np.linalg.norm(b[:])
        # y = irfft(rfft(a) * rfft(b[::-1])) / nom
        y = irfft(rfft(a) * rfft(b[::-1]))
        return y

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

    def receiveMicData(self, msg):
        self.boat.m1.receiveData = self.dataUpdate(self.boat.m1.receiveData, self.changeToVolt(msg.data_ch1))
        self.boat.m2.receiveData = self.dataUpdate(self.boat.m2.receiveData, self.changeToVolt(msg.data_ch2))
        self.flag_getdata = True
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

    def butter_bandpass(self, lowcut, highcut, fs, order):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = sg.butter(order, [low, high], btype='bandpass')
        return b, a

    def butter_bandpass_filter(self, data, lowcut, highcut, fs=192000, order=5):
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = sg.lfilter(b, a, data)
        return y
        
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))                    
            

if __name__ == '__main__':
    rospy.init_node('tdoa', anonymous=False)
    tdoa = TDOA()
    rospy.on_shutdown(tdoa.onShutdown)
    rospy.spin()
