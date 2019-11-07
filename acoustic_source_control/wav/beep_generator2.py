#!/usr/bin/python
# coding=UTF-8
# code ref: http://zwindr.blogspot.com/2017/03/python-pyaudio.html

import numpy as np
import pyaudio
import os
import sys

'''
        duration_ms
      <--->
       ___                  ___                  ___
      |   |                |   |                |   |
      |   |                |   |                |   |
______|   |________________|   |________________|   |______
      <-------------------->
            1/play_freq
'''
class Beep(object):
    def __init__(self, nchannels=1, fs=96000, duration_ms=100,
        sound_freq=10000, play_freq=0.5):
        self.nchannels = nchannels
        self.fs = fs
        self.duration_ms = duration_ms
        self.sound_freq = sound_freq
        self.play_freq = play_freq
        self.silence_ms = 1000/play_freq - self.duration_ms

        self.audio_data = np.arange(1)
        self.audio_handler = pyaudio.PyAudio()

        if os.uname()[4]=='armv7l':
            self.stream = self.audio_handler.open(format=pyaudio.paFloat32,
                    channels=nchannels, rate=fs, output=True, output_device_index=2)
        else:
            self.stream = self.audio_handler.open(format=pyaudio.paFloat32,
                    channels=nchannels, rate=fs, output=True)

    def __del__(self):
        self.audio_handler.terminate()
        self.stream.close()

    def sine(self):
        n = int(self.duration_ms/1000.0 * self.fs)
        interval = 2 * np.pi * self.sound_freq / self.fs
        return np.sin(np.arange(n) * interval)

    def silence(self):
        """
        Adding silence is easy - we add zeros to the end of our array
        """
        print self.silence_ms
        n = int(self.silence_ms/1000.0 * self.fs)
        interval = 2 * np.pi * self.sound_freq / self.fs
        return np.arange(n) * 0
 
    def generate_tone(self):
        self.audio_data = self.sine()
        self.audio_data = np.append(self.audio_data, self.silence())

    def play_tone(self):
        self.stream.write(self.audio_data.astype(np.float32).tostring())
        
 
if __name__ == '__main__':
    if len(sys.argv) == 5:
        duration_ms = int(sys.argv[1])
        sound_freq = int(sys.argv[2])
        play_freq = float(sys.argv[3])
        one_shot = int(sys.argv[4])
    else:
        print "\033[91mIncorrect number of arguement\033[0m"
        sys.exit(1)

    if sound_freq <=0:
        print "\033[91mIncorrect sound frequency\033[0m"
        sys.exit(1)

    if play_freq <= 0 or play_freq > 2:
        print "\033[91mIncorrect sound playing frequency\033[0m"
        sys.exit(1)

    beep = Beep(duration_ms=duration_ms, sound_freq=sound_freq, play_freq=play_freq)
    beep.generate_tone()

    beep.play_tone()

    while one_shot == 0:
        beep.play_tone()
