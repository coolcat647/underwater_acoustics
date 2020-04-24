#!/usr/bin/env python
import rospy
import csv
import datetime
import rospkg
import os
from robotx_msgs.msg import HydrophoneData

COLOR_RED = '\033[0;31m'
COLOR_GREEN = '\033[0;32m'
COLOR_YELLOW = '\033[0;33m'
COLOR_NC = '\033[0m'

class DataSavingNode(object):
    def __init__(self):
        # ROS subscriber and publisher
        self.sub = rospy.Subscriber("/hydrophone_data", \
                                    HydrophoneData, \
                                    self.sound_cb, \
                                    queue_size=10)
        
        # Log file path setup
        default_dir = os.path.join(rospkg.RosPack().get_path('acoustic_sampling'), "csv_files")
        if not os.path.isdir(default_dir):
            os.mkdir(default_dir)
        default_logname = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_acoustic.csv")
        self.log_path = rospy.get_param('~log', os.path.join(default_dir, default_logname))
        rospy.loginfo(COLOR_GREEN + "csv file will be saved to: " + self.log_path + COLOR_NC)
        
        # Flag for creating log file at first time
        self.flag_first = True


    # It will be run when the subscriber get the message
    def sound_cb(self, sound_msg):
        if self.flag_first:
            open(self.log_path, 'w')
            self.flag_first = False

        rospy.loginfo('Size of received data = %d samples * 4 channels', sound_msg.length)

        # Combine the tuples and transpose
        data_to_write = zip(sound_msg.data_ch1, sound_msg.data_ch2, sound_msg.data_ch3, sound_msg.data_ch4)

        # write file
        with open(self.log_path, 'a') as myFile:
            writer = csv.writer(myFile, delimiter=',')
            writer.writerows(data_to_write)

if __name__ == '__main__':
    rospy.init_node('acoustic_data_saving_node', anonymous=True)
    node = DataSavingNode()
    rospy.spin()
