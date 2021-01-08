#!/usr/bin/env python

import rospy
import numpy as np
import message_filters as mf
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
from util import *



if __name__ == "__main__":

    rospy.init_node('camera_lidar_calibration', anonymous=True)
    
    velodyne_topic = '/points_front'
    camera = '/vds_node_localhost_2218/image_raw'
    calibration = Util(velodyne_topic, camera)
    calibration.Callback() 
    rospy.spin()