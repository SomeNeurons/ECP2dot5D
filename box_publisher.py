from frame import Object
from transformations import Transformation


from threading import Thread
import pickle

import sensor_msgs.point_cloud2 as pc2
import sensor_msgs
import rospy
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class RosPublisher:

    pub_pc = None
    pub_vru_markers = None

    @staticmethod
    def init_publisher_node():
        if RosPublisher.pub_pc is None:
            rospy.init_node('publisher', anonymous=True)
            RosPublisher.pub_pc = rospy.Publisher('lidar_hdl64_roof', pc2.PointCloud2, queue_size=10)
            RosPublisher.pub_vru_markers = rospy.Publisher('vrus', Marker, queue_size=10)


    def __init__(self, frame, transformation, pc_pkl_path):
        self.running = True
        self.objs = frame.z_objects
        self.tranformation = transformation
        self.ros_ts_now = rospy.Time()
        self.pc_pkl_path = pc_pkl_path

        self.vru_marker_type = Marker.CUBE


    def start(self):
        self.p = Thread(target=self.publish_to_ros)
        self.p.start()


    def stop(self):
        self.running = False


    def publish_to_ros(self):
        rate = rospy.Rate(10)  # 10hz
        print("ROS-publish: {}".format(self.pc_pkl_path))
        pc_sample = pickle.load(open(self.pc_pkl_path, 'rb'))


        while self.running and not rospy.is_shutdown():
            self.ros_ts_now = rospy.Time.now()
            pc_sample.header.stamp = self.ros_ts_now  # otherwise rviz will not show the pc
            RosPublisher.pub_pc.publish(pc_sample)

            self.tranformation.publish_tf(self.ros_ts_now)
            self.publish_markers()

            rate.sleep()


    def transform_marker_for_obj(self, obj, marker):
        obj_yaw_in_camera_frame = obj.obj_yaw_in_camera_frame
        if obj_yaw_in_camera_frame is None:
            obj_yaw_in_camera_frame = 0

        marker_width = obj.real_width
        if marker_width is None:
            marker_width = 1.0

        marker_length = obj.real_length
        if marker_length is None:
            marker_length = 1.0

        marker_height = obj.real_height
        if marker_height is None:
            marker_height = 1.0

        middle = obj.get_obj_center_from_unrectified_and_distance()

        # orienation of the object in camera_frame is relative orienation to camera ray of sight
        # plus the orientation from the camera ray of sight from Z axis (depth in camera)
        q = quaternion_from_euler(0, obj_yaw_in_camera_frame, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.pose.position.x = middle[0]
        marker.pose.position.y = middle[1]
        marker.pose.position.z = middle[2]

        marker.scale.x = marker_length
        marker.scale.y = marker_height
        marker.scale.z = marker_width

        marker.type = self.vru_marker_type

        return True


    def transform_orientation_marker_for_obj(self, obj, marker):
        if obj.distance is None:
            return False

        distance = obj.distance

        unrectified_cx = (obj.x0 + obj.x1) / 2.0
        unrectified_cy = obj.y0
        top_middle = self.tranformation.getOpticalCoordinates((unrectified_cx, unrectified_cy), distance)

        obj_yaw_in_camera_frame = obj.obj_yaw_in_camera_frame
        if obj_yaw_in_camera_frame is None:
            obj_yaw_in_camera_frame = 0

        q = quaternion_from_euler(0, obj_yaw_in_camera_frame, 0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.pose.position.x = top_middle[0]
        marker.pose.position.y = top_middle[1]
        marker.pose.position.z = top_middle[2]

        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.type = Marker.ARROW

        return True


    def generate_all_markers_for_obj(self, obj, marker):
        marker.color.a = 0.75

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        yield self.transform_marker_for_obj(obj, marker)
        yield self.transform_orientation_marker_for_obj(obj, marker)


    def get_base_marker(self):
        vru = Marker()
        vru.header.frame_id = "cam_stereo_ar0230_left_optical"
        vru.header.stamp = rospy.Time.now()
        vru.ns = "vru_markers"
        vru.action = vru.DELETEALL
        vru.type = self.vru_marker_type
        vru.id = 0

        vru.scale.x = 0.3
        vru.scale.y = 0.3
        vru.scale.z = 0.3

        vru.pose.position.x = 5
        vru.pose.position.y = 0
        vru.pose.position.z = 0
        vru.pose.orientation.x = 0.0
        vru.pose.orientation.y = 0.0
        vru.pose.orientation.z = 0.0
        vru.pose.orientation.w = 1.0

        return vru


    def publish_markers(self):
        vru = self.get_base_marker()
        # delete all from earlier frame
        RosPublisher.pub_vru_markers.publish(vru)
        vru.action = vru.ADD

        marker_id = 0
        for index, obj in enumerate(self.objs):
            obj.marker_ids = []
            for valid in self.generate_all_markers_for_obj(obj, vru):
                if valid:
                    vru.id = marker_id
                    obj.marker_ids.append(marker_id)
                    marker_id += 1
                    RosPublisher.pub_vru_markers.publish(vru)

