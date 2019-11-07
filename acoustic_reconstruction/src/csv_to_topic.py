#!/usr/bin/env python
import sys
import os
import rospy
import rospkg
import csv
import datetime
from robotx_msgs.msg import HydrophoneData
from std_msgs.msg import Header

FRAME_ID = "base_link"
Fs = 96000

class Csv2TopicNode(object):
    def __init__(self, csv_path):
        # ROS subscriber and publisher
        self.pub = rospy.Publisher('/hydrophone_data', HydrophoneData, queue_size = 10)
        self.sound_msg = HydrophoneData()

        # Check whether the csv file is existing.
        self.csv_path = csv_path
        if not os.path.isfile(self.csv_path) or not self.csv_path.lower().endswith(('.csv')):
            rospy.logerr('The file \'{}\' is not a csv file or not in the directory.'.format(self.csv_path))
            exit(-1)

    def run(self):
        rate = rospy.Rate(2.0) # 2hz
        cnt = 0
        while not rospy.is_shutdown():
            rospack = rospkg.RosPack()
            self.csvfile = open(self.csv_path, 'rb')

            for row in csv.reader(self.csvfile, delimiter=','):
                self.sound_msg.data_ch1.append(int(row[0]))
                self.sound_msg.data_ch2.append(int(row[1]))
                self.sound_msg.data_ch3.append(int(row[2]))
                self.sound_msg.data_ch4.append(int(row[3]))
                cnt = cnt + 1
                if(cnt == Fs):
                    cnt = 0
                    self.sound_msg.length = Fs
                    self.sound_msg.header = Header(frame_id=FRAME_ID, stamp=rospy.Time.now())
                    self.pub.publish(self.sound_msg)

                    self.sound_msg.data_ch1.clear()
                    self.sound_msg.data_ch2.clear()
                    self.sound_msg.data_ch3.clear()
                    self.sound_msg.data_ch4.clear()
                    rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node('csv_to_topic_node', anonymous = False)
    if len(sys.argv) < 2:
        rospy.logerr('Usage: rosrun acoustic_reconstruction csv_to_topic.py FILE_PATH')
        exit(-1)

    node = Csv2TopicNode(sys.argv[1])
    node.run()
