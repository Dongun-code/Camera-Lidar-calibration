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
        # print('velodyne shape:',original_point[0])
        # inrange = np.where(original_point[:,2] > 0 & original_point[:,2] < 300)


        # 3D_point = original_point[inrange]
        return original_point


    def projectionPoint(self, points, camera_info):

        self.camera_model.fromCameraInfo(camera_info)

        points2D = [self.camera_model.project3dToPixel(point) for point in points[:, :3]]
        points2D = np.asarray(points2D)
        return points2D




    def lidar2imgProjection(self, lidar, camera_info):

        # if self.h_fov is None:
        #     self.h_fov = (-50, 50)
        # if self.h_fov[0] < -50:
        #     self.h_fov = (-50,) + self.h_fov[1:]
        # if self.h_fov[1] > 50:
        #     self.h_fov = self.h_fov[:1] + (50,)

        point3D = self.transfromPoint(lidar)
        point2D = self.projectionPoint(point3D, camera_info)
        print(point2D.shape)




    def projectionProcess(self, lidar, camera, camera_info):

        #   camera topic
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(camera, "bgr8")
        except CvBridgeError, e:
            print(e)

        # msg = self.bridge.cv2_to_imgmsg(cv2_img)
        self.lidar2imgProjection(lidar, camera_info)
        # pub.publish(msg)
        # print('long')



    def Callback(self):
        print("hi")
        mf_function = mf.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub, self.camera_info], 10, 0.1)
        mf_function.registerCallback(self.projectionProcess)

