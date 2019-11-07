#!/usr/bin/env python

import rospy
import rospkg
import os
import time
import signal
import subprocess
from pinger_ctrl.srv import PingerTrigger, PingerTriggerResponse
# For Rpi3 relay controll
if os.uname()[4] == 'armv7l':
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)


class Pinger(object):
    def __init__(self, node_name, duration_ms=50, sound_freq=15000, play_freq=0.5):
        self.srv = rospy.Service(node_name, PingerTrigger, self.sound_handler)
        rospack = rospkg.RosPack()
        self.pack_path = rospack.get_path('pinger_ctrl')
        self.cmd = ''
        self.process = None             # declare for playsound process handler

        # Declare pinger parameters with default setting, then import the user setting if they were existed
        self.duration_ms = duration_ms
        self.sound_freq  = sound_freq
        self.play_freq   = play_freq
        if rospy.has_param('~duration_ms'):
            self.duration_ms = rospy.get_param('~duration_ms')
        if rospy.has_param('~sound_freq'):
            self.sound_freq = rospy.get_param('~sound_freq')
        if rospy.has_param('~play_freq'):
            self.play_freq = rospy.get_param('~play_freq')


        # For Rpi3 relay controll
        if os.uname()[4] == 'armv7l':   
            GPIO.setup(7, GPIO.OUT, initial = GPIO.LOW)

        rospy.loginfo('%s is waitting for request' % rospy.get_name() )

    # Service handler
    def sound_handler(self, req):
        try:
            '''
                If there is any old process existed when you call the service,
                send the signal to kill the process first.
            '''
            if self.process != None:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)    
                rospy.loginfo("stop %s " % rospy.get_name()) 
                self.process = None
                if os.uname()[4] == 'armv7l':
                    GPIO.output(7, GPIO.LOW) # Turn off the amplifier

            # The pinger will just play one time in one_shot mode
            self.cmd = 'python %s/wav/beep_generator2.py %s %s %s %s' % \
                (self.pack_path, self.duration_ms, self.sound_freq, self.play_freq, req.one_shot)
            
            if req.on_off != 0:
                rospy.loginfo("Output freqency of %s = %d Hz with %d ms and %d ms silence" % \
                (rospy.get_name(), self.sound_freq, self.duration_ms, 1000 / self.play_freq - self.duration_ms))

                # For Rpi3 relay controll
                if os.uname()[4] == 'armv7l':   
                    GPIO.output(7, GPIO.HIGH)   # Turn on the amplifier and wait for ready
                    for cnt in range(10, 0, -1):
                        rospy.loginfo('Waitting for amplifier ready in %d sec...' % cnt)
                        time.sleep(1)
                
                # Open the subprocess
                self.process = subprocess.Popen(self.cmd, stdout= subprocess.PIPE, shell=True, preexec_fn=os.setsid)
                return PingerTriggerResponse('playing the sound...')

        except OSError as e:
            print "OSError > ", e.errno
            print "OSError > ", e.strerror
            print "OSError > ", e.filename
        except:
            print "Error > ",sys.exc_info()[0]
        return PingerTriggerResponse('[%s] stop...' % rospy.get_name())

    def onShutdown(self):
        # This program must get here after killing the ros node
        if pinger.process != None:
            os.killpg(os.getpgid(pinger.process.pid), signal.SIGTERM)
            rospy.loginfo('[%s] stop...' % rospy.get_name()) 
            pinger.process = None
        if os.uname()[4] == 'armv7l':
            GPIO.output(7, GPIO.LOW)    # Just turn off the relay for safety reason
        rospy.loginfo('[%s] Shutdown.' % rospy.get_name())


if __name__ == "__main__":
    if sys.argv[1].find('num=') != -1:
        pinger_num = sys.argv[1][4] 
    else: 
        print "\033[91mPinger number has not be assigned.\033[0m"
        exit(-1)

    node_name = 'pinger' + pinger_num

    rospy.init_node(node_name, anonymous = False)
    pinger = Pinger(node_name = node_name)
    rospy.on_shutdown(pinger.onShutdown)
    rospy.spin()

    
