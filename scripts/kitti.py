#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import cv2
import os
from cv_bridge import CvBridge
import numpy as np

import sensor_msgs.point_cloud2 as pcl2

from std_msgs.msg import Header


DATA_PATH = '/home/ubuntu1604/data/kitti/RawData/2011_09_26/2011_09_26_drive_0005_sync/'

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node', anonymous=True)
    cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        img=cv2.imread(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
        point_cloud=np.fromfile(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame), dtype=np.float32).reshape(-1,4)


        hello_str = "image publish %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"


        pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:,:3]))
        rate.sleep()
        frame += 1
        frame %= 154
