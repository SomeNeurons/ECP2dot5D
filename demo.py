import os
import time
import cv2

from frame import Frame
from city_manager import CityManager
from box_publisher import RosPublisher


class Demo:

    def __init__(self, city, label, img, pc):
        cm = CityManager()
        transformation = cm.get_transformation_for_city(city)

        frame = Frame(label, transformation)

        self.ros_publisher = RosPublisher(frame, transformation, pc)

        self.im = cv2.imread(img)


    def start(self):
        self.ros_publisher.start()
        cv2.imshow('img', self.im)


    def stop(self):
        self.ros_publisher.stop()


if __name__ == '__main__':

    RosPublisher.init_publisher_node()

    for index in range(1, 60):
        city = 'amsterdam'
        label = './resources/data/day/labels/train/amsterdam/amsterdam_{:05d}.json'.format(index)
        img = './resources/data/day/imgs/train/amsterdam/amsterdam_{:05d}.png'.format(index)
        pc = './resources/data/day/pcs/train/amsterdam/amsterdam_{:05d}.pkl'.format(index)

        demo = Demo(city, label, img, pc)
        demo.start()
        cv2.waitKey()
        demo.stop()



