#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import signal
import numpy as np
from numpy import arange, sin, cos, pi
from scipy.fftpack import fft
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import rospy
from robotx_msgs.msg import HydrophoneData

COLOR_NC='\033[0m'
COLOR_GREEN='\033[0;32m'
COLOR_RED='\033[0;31m'

class Plot2D():
    def __init__(self):
        self.traces = dict()

        QtGui.QApplication.setGraphicsSystem('raster')
        self.app = QtGui.QApplication([])

        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000, 900)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        self.canvas_list = []
        self.canvas_list.append(self.win.addPlot(title="Channel 1", row=1, col=1))
        self.canvas_list.append(self.win.addPlot(title="Channel 2", row=2, col=1))
        self.canvas_list.append(self.win.addPlot(title="Channel 3", row=3, col=1))
        self.canvas_list.append(self.win.addPlot(title="Channel 4", row=4, col=1))
        self.spectrum_list = []
        self.spectrum_list.append(self.win.addPlot(title='Spectrum 1', row=1, col=2))
        self.spectrum_list.append(self.win.addPlot(title='Spectrum 2', row=2, col=2))
        self.spectrum_list.append(self.win.addPlot(title='Spectrum 3', row=3, col=2))
        self.spectrum_list.append(self.win.addPlot(title='Spectrum 4', row=4, col=2))

        self.num_sample = 192000
        self.fs = 192000.0
        self.t_range = np.arange(0, self.num_sample / self.fs, 1/self.fs)
        self.f_range = np.linspace(0, self.fs / 2, self.num_sample / 2)

        self.ch1_data = []
        self.ch2_data = []
        self.ch3_data = []
        self.ch4_data = []
        self.ch1_data_old = []
        self.ch2_data_old = []
        self.ch3_data_old = []
        self.ch4_data_old = []
        self.sub = rospy.Subscriber("hydrophone_data", \
                                    HydrophoneData, \
                                    self.sound_cb, \
                                    queue_size=10)
        

    def sound_cb(self, msg):
        ch1 = [float(i)/2147483648 for i in msg.data_ch1]
        ch2 = [float(i)/2147483648 for i in msg.data_ch2]
        ch3 = [float(i)/2147483648 for i in msg.data_ch3]
        ch4 = [float(i)/2147483648 for i in msg.data_ch4]

        self.ch1_data = ch1 + self.ch1_data_old
        self.ch2_data = ch2 + self.ch2_data_old
        self.ch3_data = ch3 + self.ch3_data_old
        self.ch4_data = ch4 + self.ch4_data_old
        self.ch1_data_old = ch1
        self.ch2_data_old = ch2
        self.ch3_data_old = ch3
        self.ch4_data_old = ch4


    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def trace(self, name, data_x, data_y):
        if name in self.traces:
            self.traces[name].setData(data_x, data_y)
        else:
            if name.find("ch") != -1:
                index = int(name[-1])-1
                self.traces[name] = self.canvas_list[index].plot(pen='c', width=1)
                self.canvas_list[index].setYRange(-0.05, 0.05, padding=0)
            elif name.find("spectrum") != -1:
                index = int(name[-1])-1
                self.traces[name] = self.spectrum_list[index].plot(pen='m', width=2)
                self.spectrum_list[index].setLogMode(x=True, y=False)

    def update(self):
        if len(self.ch1_data) > 0:
            # time domain
            self.trace("ch1", self.t_range, self.ch1_data)
            self.trace("ch2", self.t_range, self.ch2_data)
            self.trace("ch3", self.t_range, self.ch3_data)
            self.trace("ch4", self.t_range, self.ch4_data)

            # frequency domain
            sp_data1 = fft(np.array(self.ch1_data, dtype='float32'))
            sp_data1 = np.abs(sp_data1[0:int(self.num_sample / 2)])
            self.trace("spectrum1", self.f_range, sp_data1)
            sp_data2 = fft(np.array(self.ch2_data, dtype='float32'))
            sp_data2 = np.abs(sp_data2[0:int(self.num_sample / 2)])
            self.trace("spectrum2", self.f_range, sp_data2)
            sp_data3 = fft(np.array(self.ch3_data, dtype='float32'))
            sp_data3 = np.abs(sp_data3[0:int(self.num_sample / 2)])
            self.trace("spectrum3", self.f_range, sp_data3)
            sp_data4 = fft(np.array(self.ch4_data, dtype='float32'))
            sp_data4 = np.abs(sp_data4[0:int(self.num_sample / 2)])
            self.trace("spectrum4", self.f_range, sp_data4)


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    if 'armv7' in os.uname()[-1]:
        print(COLOR_RED + 'The machine doesn\'t support this function.' + COLOR_NC)
        exit(-1)

    rospy.init_node('acoustic_data_visaulization_node', anonymous=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    p2d = Plot2D()
    timer = QtCore.QTimer()
    timer.timeout.connect(p2d.update)

    # Timer interval = 2 ms
    timer.start(500)
    p2d.start()
