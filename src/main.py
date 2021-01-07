import numpy as np
import rospy
import message_filters as mf
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

class calibrationSensors:
    def __init__():
        self.velodyne_topic = ''
        self.camera = ''
        self.lidar_sub = mf.Subscriber(self.velodyne_topic. PointCloud2)
        self.camera_sub = mf.Subscriber(self.camera, Image)
        self.bridge = CvBridge()

    def lidarProjection(self, lidar):






    def transform(self, lidar, camera):

        #   camera topic
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(camera, "bgr8")
        except CvBridgeError, e:
            print(e)

            self.lidarProjection()


    def Callback(self):
        mf_function = mf.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub], 10, 0.1)
        mf_function.registerCallback(self.transform)









if __name__ == "main":
    rospy.init_node('camera_lidar_calibration')
    calibration = calibrationSensors()
    calibration.main()
    
    rospy.spin()