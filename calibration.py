# ROS Packages
import geometry_msgs
import image_geometry
from image_geometry import PinholeCameraModel
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud, transform_to_kdl
from tf.transformations import quaternion_from_euler
import tf2_ros
import rospy
import PyKDL

import pickle


class Calibration:

    def __init__(self, tour):
        self.tour = tour
        self.base_link_to_body = self.init_base_link_to_body()
        self.body_to_cam = self.get_body_to_cam_trafo(self.tour)
        self.pc_to_cam = self.get_pc_to_cam_trafo(self.tour)
        self.cam_info = self.get_cam_info(self.tour)
        self.pcm = self.get_pcm(self.cam_info)


    def init_base_link_to_body(self):
        base_link_to_body = geometry_msgs.msg.TransformStamped()
        base_link_to_body.header.frame_id = 'base_link'
        base_link_to_body.child_frame_id = 'body'

        base_link_to_body.transform.translation.x = 0
        base_link_to_body.transform.translation.y = 0
        base_link_to_body.transform.translation.z = 0.35

        quad = quaternion_from_euler(0.0, 0.0, 0.0)

        base_link_to_body.transform.rotation.x = quad[0]
        base_link_to_body.transform.rotation.y = quad[1]
        base_link_to_body.transform.rotation.z = quad[2]
        base_link_to_body.transform.rotation.w = quad[3]

        return base_link_to_body


    def init_body_to_cam_trafo(self, twist, quad):
        body_to_cam = geometry_msgs.msg.TransformStamped()
        body_to_cam.header.frame_id = 'body'
        body_to_cam.child_frame_id = 'cam_stereo_ar0230_left_optical'

        body_to_cam.transform.translation.x = twist[0]
        body_to_cam.transform.translation.y = twist[1]
        body_to_cam.transform.translation.z = twist[2]

        body_to_cam.transform.rotation.x = quad[0]
        body_to_cam.transform.rotation.y = quad[1]
        body_to_cam.transform.rotation.z = quad[2]
        body_to_cam.transform.rotation.w = quad[3]

        return body_to_cam


    def init_pc_to_cam_trafo(self, twist, quad):
        pc_to_cam = geometry_msgs.msg.TransformStamped()
        pc_to_cam.header.frame_id = 'cam_stereo_ar0230_left_optical'
        pc_to_cam.child_frame_id = 'lidar_hdl64_roof'

        pc_to_cam.transform.translation.x = twist[0]
        pc_to_cam.transform.translation.y = twist[1]
        pc_to_cam.transform.translation.z = twist[2]

        pc_to_cam.transform.rotation.x = quad[0]
        pc_to_cam.transform.rotation.y = quad[1]
        pc_to_cam.transform.rotation.z = quad[2]
        pc_to_cam.transform.rotation.w = quad[3]

        return pc_to_cam


    def get_base_link_to_body(self):
        return self.init_base_link_to_body()


    def get_body_to_cam_trafo(self, tour):
        if tour in ['IT', 'ND', 'EE', 'DP']:
            twist = [2.085160, 0.134792, 0.890600]
            quad = [-0.49100383522562174, 0.48476408062482257, -0.5080411841043767, 0.5155707276247591]
        elif tour in ['D', 'SF']:
            twist = [2.085160, 0.134792, 0.890600]
            quad = [0.5124529570506265, -0.5069110343007684, 0.48697348943761243, -0.49324435191814364]
        else:
            assert False

        return self.init_body_to_cam_trafo(twist, quad)


    def get_pc_to_cam_trafo(self, tour):
        if tour == 'IT':
            twist = [0.10933691719362493, -0.4939558191651061, -0.7693898142960587]
            quad = [0.475323992692665, -0.4752959344166986, 0.523011865497655, 0.5240414728304658]
        elif tour == 'D':
            twist = [0.11092079088778384, -0.42524533497149875, -0.809178955685828]
            quad = [-0.4979235149720077, 0.49732944915298044, -0.5016090485697907, -0.5031142560453077]
        elif tour == 'EE':
            twist = [0.10933691719362493, -0.4939558191651061, -0.7693898142960587]
            quad = [0.475323992692665, -0.4752959344166986, 0.523011865497655, 0.5240414728304658]
        elif tour == 'DP':
            twist = [0.10933691719362493, -0.4939558191651061, -0.7693898142960587]
            quad = [0.475323992692665, -0.4752959344166986, 0.523011865497655, 0.5240414728304658]
        elif tour == 'ND':
            twist = [0.10933691719362493, -0.4939558191651061, -0.7693898142960587]
            quad = [0.475323992692665, -0.4752959344166986, 0.523011865497655, 0.5240414728304658]
        elif tour == 'SF':
            twist = [0.11092079088778384, -0.42524533497149875, -0.809178955685828]
            quad = [0.4971412955817936, -0.49811139262639803, 0.5023260844862588, 0.5023983256418658]

        return self.init_pc_to_cam_trafo(twist, quad)


    def get_cam_info(self, tour):
        cam_info_path = './resources/calibration/{}/camera_info.pkl'.format(tour)
        cam_info = pickle.load(open(cam_info_path, 'rb'))
        return cam_info


    def get_pcm(self, cam_info):
        pcm = image_geometry.PinholeCameraModel()
        pcm.fromCameraInfo(cam_info)
        return pcm
