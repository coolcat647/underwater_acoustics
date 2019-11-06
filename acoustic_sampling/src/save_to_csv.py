#!/usr/bin/env python
import rospy
import csv
import datetime
from robotx_msgs.msg import HydrophoneData

class DataSavingNode(object):
    def __init__(self):
        # ROS subscriber and publisher
        self.sub = rospy.Subscriber("/hydrophone_data", HydrophoneData, self.sound_cb, queue_size=5)

        # Create the data log file which is named by date
        # The log data will be saved where you call the command 'rosrun'
        now = datetime.datetime.now()
        self.log_filename = now.strftime("%Y%m%d_%H%M%S_acoustic.csv")
        open(self.log_filename, 'w') 


    # It will be run when the subscriber get the message
    def sound_cb(self, sound_msg):
        rospy.loginfo('Length of received data = %d samples', sound_msg.length)

        # Combine the tuples and transpose
        data_to_write = zip(sound_msg.data_ch1, sound_msg.data_ch2, sound_msg.data_ch3, sound_msg.data_ch4)

        # write file
        with open(self.log_filename, 'a') as myFile:
            writer = csv.writer(myFile, delimiter=',')
            writer.writerows(data_to_write)

if __name__ == '__main__':
    rospy.init_node('acoustic_data_saving_node', anonymous=True)
    node = DataSavingNode()
    rospy.spin()
