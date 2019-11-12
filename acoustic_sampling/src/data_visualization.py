#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
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
        self.i = 0
        self.traces = dict()

        #QtGui.QApplication.setGraphicsSystem('raster')
        self.app = QtGui.QApplication([])
        #mw = QtGui.QMainWindow()
        #mw.resize(800,800)

        self.win = pg.GraphicsWindow(title="Basic plotting examples")
        self.win.resize(1000, 900)
        self.win.setWindowTitle('pyqtgraph example: Plotting')

        # Enable antialiasing for prettier plots
        pg.setConfigOptions(antialias=True)

        self.canvas = self.win.addPlot(title="Pytelemetry", row=1, col=1,)
        self.spectrum = self.win.addPlot(title='Spectrum', row=2, col=1)

        self.num_sample = 1024
        self.fs = float(1000)
        self.t_range = np.arange(0, self.num_sample/self.fs, 1/self.fs)
        self.f_range = np.linspace(0, self.fs / 2, self.num_sample / 2)


    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def trace(self,name,data_x,data_y):
        if name in self.traces:
            self.traces[name].setData(data_x,data_y)
        else:
            # self.traces[name] = self.canvas.plot(pen='y')
            if name == 'sin':
                self.traces[name] = self.canvas.plot(pen='c', width=3)
                # self.waveform.setYRange(0, 255, padding=0)
                # self.waveform.setXRange(0, 2 * self.CHUNK, padding=0.005)
            if name == 'spectrum':
                self.traces[name] = self.spectrum.plot(pen='m', width=3)
                # self.spectrum.setLogMode(x=True, y=True)
                # self.spectrum.setYRange(-4, 0, padding=0)
                # self.spectrum.setXRange(
                    # np.log10(20), np.log10(self.RATE / 2), padding=0.005)

    def update(self):
        sine_data = sin(2 * pi * self.t_range * 100 + self.i)
        self.trace("sin", self.t_range, sine_data)

        sp_data = fft(np.array(sine_data, dtype='float32'))
        sp_data = np.abs(sp_data[0:int(self.num_sample / 2)])
        self.trace("spectrum", self.f_range, sp_data)

    
        self.i += 0.1

        

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    if 'armv7' in os.uname()[-1]:
        print(COLOR_RED + 'The machine doesn\'t support this function.' + COLOR_NC)
        exit(-1)

    p2d = Plot2D()
    timer = QtCore.QTimer()
    timer.timeout.connect(p2d.update)
    timer.start(2)
    p2d.start()