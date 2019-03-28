#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2

DATA_PATH = '/home/ubuntu1604/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync'

if __name__ == '__main__':
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        cam_pub.publish(hello_str)
        rate.sleep()
