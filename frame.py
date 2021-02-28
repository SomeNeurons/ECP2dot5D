import json
import numpy as np
import math


class Frame:

    def __init__(self, label_file, transformation):
        self.z_objects = []
        for o in json.load(open(label_file, 'r'))['children']:
            if '3dp' in o:
                self.z_objects.append(Object(o, transformation))


class Object:

    def __init__(self, json_dict, transformation):
        self.json_dict = json_dict

        self.x0 = json_dict['x0']
        self.x1 = json_dict['x1']
        self.y0 = json_dict['y0']
        self.y1 = json_dict['y1']

        self.x = json_dict['3dp']['x']
        self.y = json_dict['3dp']['y']
        self.z = json_dict['3dp']['z']

        self.distance = self.z

        self.transformation = transformation

        self.real_width = None
        self.real_length = None
        self.real_height = None
        self.obj_yaw_in_camera_frame = None

        self.calculate_real_world_values()


    def calculate_real_world_values(self):
        self.calculate_real_extents_for_distance()
        self.calculate_orientation_in_camera_frame()

    def calculate_real_extents_for_distance(self):
        topleft = self.transformation.getOpticalCoordinates((self.x0, self.y0), self.distance)
        bottomright = self.transformation.getOpticalCoordinates((self.x1, self.y1), self.distance)

        self.real_height = bottomright[1] - topleft[1]
        self.real_width = bottomright[0] - topleft[0]

        self.real_length = self.get_width_length_ratio() * self.real_width


    def calculate_orientation_in_camera_frame(self):
        # orienation of the object in world_frame is relative orienation to camera ray of sight
        # plus the orientation from the camera ray of sight from Z axis (depth in camera)
        obj_middle = self.get_obj_center_from_unrectified_and_distance()
        obj_orientation = self.get_orientation()
        if obj_orientation is None:
            return None

        orientation_to_ray_of_sight = np.deg2rad(obj_orientation)
        rotation_from_ray_of_sight = math.atan2(obj_middle[0], obj_middle[2])
        # Add Pi/2 since 0 deg is the X-Axis (so to the right)
        self.obj_yaw_in_camera_frame = 0.5 * math.pi + rotation_from_ray_of_sight + orientation_to_ray_of_sight


    def is_rider(self):
        return self.json_dict.get('identity', None) is not None and self.json_dict['identity'] == 'rider'


    def get_width_length_ratio(self):
        # Return ratio length/width
        # Currently we assume pedestrians to be symetrical
        # Riders are longer than wide (obviously)
        if self.is_rider():
            return 2.5
        else:
            return 1.0


    def get_obj_center_from_unrectified_and_distance(self):
        unrectified_cx = (self.x0 + self.x1) / 2.0
        unrectified_cy = (self.y0 + self.y1) / 2.0
        z = self.distance
        middle = self.transformation.getOpticalCoordinates((unrectified_cx, unrectified_cy), z)

        return middle


    def get_orientation(self):
        if self.is_rider():
            for c in self.json_dict['children']:
                if 'Orient' in c:
                    return c['Orient']
            return None
        else:
            return self.json_dict.get('Orient', None)