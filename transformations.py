import sys

import pickle
import numpy as np

# ROS Packages
import geometry_msgs
import image_geometry
from image_geometry import PinholeCameraModel
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud, transform_to_kdl
from tf.transformations import quaternion_from_euler
import tf2_ros
import rospy
import PyKDL


###
from calibration import Calibration


class Transformation:

    def __init__(self, tour):
        self.tour = tour
        self.calibration = Calibration(tour)


    def broadcast_tf_trafo(self, tf_trafo, ros_ts=rospy.Time()):
        br = tf2_ros.TransformBroadcaster()
        tf_trafo.header.stamp = ros_ts
        br.sendTransform(tf_trafo)


    def publish_tf(self, ros_ts=rospy.Time()):
        br = tf2_ros.TransformBroadcaster()
        for trafo in [self.calibration.base_link_to_body, self.calibration.body_to_cam, self.calibration.pc_to_cam]:
            trafo.header.stamp = ros_ts
            br.sendTransform(trafo)


    def getDistForRealHeight(self, rectified_height, real_height):
        pcm = self.calibration.pcm
        dist = pcm.fx()/rectified_height * real_height
        return dist


    def getRectifiedPoint(self, unrectified_uv):
        pcm = self.calibration.pcm
        rectified_uv = pcm.rectifyPoint(unrectified_uv)
        return rectified_uv


    def getOpticalCoordinates(self, unrectified_uv, camera_z):
        pcm = self.calibration.pcm
        rectified_uv = pcm.rectifyPoint(unrectified_uv)
        ray = pcm.projectPixelTo3dRay(rectified_uv)
        ray = np.array(ray)
        ray_len = camera_z / ray[2]
        fullray = ray_len * ray
        return fullray


    def getLidarBasedPoint(self, unrectified_uv, camera_z):
        opt_coords = self.transformation.getOpticalCoordinates(unrectified_uv, camera_z)
        trafo = self.calibration.pc_to_cam
        t_kdl = transform_to_kdl(trafo).Inverse()
        p_out = t_kdl * PyKDL.Vector(opt_coords[0], opt_coords[1], opt_coords[2])
        return p_out


    def getBaselinkToCamera(self, tour):
        base_to_body = transform_to_kdl(self.calibration.base_link_to_body)
        body_to_cam = transform_to_kdl(self.calibration.body_to_cam)
        base_to_cam = base_to_body * body_to_cam
        cam_to_base = base_to_cam.Inverse()
        return base_to_cam, cam_to_base



