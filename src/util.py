#!/usr/bin/env python
import rospy
import numpy as np
import message_filters as mf

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import image_geometry
import cv2

pub = rospy.Publisher("/calibration_image",Image, queue_size=1)

class Util:
    def __init__(self, lidar_path, img_path):

        self.velodyne_topic = lidar_path
        self.camera = img_path
        self.lidar_sub = mf.Subscriber(self.velodyne_topic, PointCloud2)
        self.camera_sub = mf.Subscriber(self.camera, Image)
        self.bridge = CvBridge()
        self.TF_BUFFER = tf2_ros.Buffer()
        self.TF_LISTENER = tf2_ros.TransformListener(self.TF_BUFFER)
        self.info_msg = '/vds_node_localhost_2218/camera_info'
        self.camera_info = mf.Subscriber( self.info_msg, CameraInfo)
        # info_msg = sensor_msgs.Subscriber(self.camera_info, CameraInfo)
        self.camera_model = image_geometry.PinholeCameraModel()


    def transfromPoint(self, lidar):

        try:
            transform_param = self.TF_BUFFER.lookup_transform('world', 'velodyne', rospy.Time())
            self.velodyne = do_transform_cloud(lidar, transform_param)
        except tf2_ros.LookupException:
            pass

        original_point = ros_numpy.point_cloud2.pointcloud2_to_array(self.velodyne)
        original_point = np.asarray(original_point.tolist())
        inrange = np.where((original_point[:,2] > 0) & (original_point[:,2] < 300))
        original_point = original_point[inrange[0]]

        return original_point


    def projectionPoint(self, points, camera_info):

        self.camera_model.fromCameraInfo(camera_info)

        points2D = [self.camera_model.project3dToPixel(point) for point in points[:, :3]]
        points2D = np.asarray(points2D)

        return points2D


    def extractDistance(self, points3D, points2D):

        # inrange = np.where((points2D[:, 0] >= 0) &
        #         (points2D[:, 1] >= 0) &
        #         (points2D[:, 0] < self.img_shape.shape[1]-1) &
        #         (points2D[:, 1] < self.img_shape.shape[0]-1))

        inrange = np.where((points2D[:, 0] >= 0) &
                (points2D[:, 1] >= 0) &
                (points2D[:, 0] < self.cv2_img.shape[1]) &
                (points2D[:, 1] < self.cv2_img.shape[0]))

        distance = points3D[inrange[0]]

        for points in distance:
            distance_map_ = [points[0]**2 + points[1]**2 + points[2]**2 ]
        
        points2D = points2D[inrange[0]].round().astype('int')

        return distance_map_, points2D



    def processPoints(self, lidar, camera_info):
        # if self.h_fov is None:
        #     self.h_fov = (-50, 50)
        # if self.h_fov[0] < -50:
        #     self.h_fov = (-50,) + self.h_fov[1:]
        # if self.h_fov[1] > 50:
        #     self.h_fov = self.h_fov[:1] + (50,)

        point3D = self.transfromPoint(lidar)
        point2D = self.projectionPoint(point3D, camera_info)
        distance_map, points2D = self.extractDistance(point3D, point2D)
        # print(point2D.shape)

        return distance_map, points2D




    def lidar2imageProjection(self, lidar, camera, camera_info):

        #   camera topic
        try:
            self.cv2_img = self.bridge.imgmsg_to_cv2(camera, "bgr8")
        except CvBridgeError, e:
            print(e)
        
        # self.img_shape = cv2_img.shape

        # msg = self.bridge.cv2_to_imgmsg(cv2_img)
        distance_map, points2D = self.processPoints(lidar, camera_info)

        for i in range(len(points2D)):
            cv2.circle(self.cv2_img, tuple(points2D[i]), 2, (0,0,255), -1)

        pub.publish(self.bridge.cv2_to_imgmsg(self.cv2_img, "bgr8"))




    def Callback(self):
        print("hi")
        mf_function = mf.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub, self.camera_info], 5, 0.1)
        mf_function.registerCallback(self.lidar2imageProjection)

