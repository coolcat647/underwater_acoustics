import numpy as np 
import math 

class boatStatus:

    def __init__(self, x, y, head, fs):
        self.x = x 
        self.y = y
        self.h = head/180*np.pi 
        self.max_sel = 140
        self.sen = -208
        self.guessAngle = 361

        #mic status
        self.m1 = micPosition(fs)
        self.m2 = micPosition(fs)
        self.m3 = micPosition(fs)

    def mic_update(self, boat_mic):
        self.m1.h = self.h-np.pi*1/3
        self.m2.h = self.h+np.pi*1/3
        self.m3.h = self.h+np.pi
        self.m1.x = round(self.x+np.sin(self.m1.h)*boat_mic,3)
        self.m1.y = round(self.y+np.cos(self.m1.h)*boat_mic,3)
        self.m2.x = round(self.x+np.sin(self.m2.h)*boat_mic,3)
        self.m2.y = round(self.y+np.cos(self.m2.h)*boat_mic,3)
        self.m3.x = round(self.x+np.sin(self.m3.h)*boat_mic,3)
        self.m3.y = round(self.y+np.cos(self.m3.h)*boat_mic,3)
        self.m1_m2_dl = math.sqrt((self.m1.x-self.m2.x)**2+(self.m1.y-self.m2.y)**2)

class micPosition:
    def __init__(self, buffer_len):
        self.x = 0.0 #mic position x  
        self.y = 0.0 #mic position y
        self.h = 0.0 #degree of mic 
        self.rt = 0.0 #ideal receive time
        self.sel = 0
        self.receiveData = np.zeros(buffer_len)

