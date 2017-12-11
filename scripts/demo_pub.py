#!/usr/bin/env python
import sys

import cv2
import glob
import rospkg
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_pub():
    def __init__(self):
        rospy.init_node("image_pub")
        rospack = rospkg.RosPack()
        self.images = glob.glob(rospack.get_path('rail_object_detector') + '/libs/darknet/data/*.jpg')
        self.rate = rospy.get_param('~rate', default=1)
        self.image_topic = rospy.Publisher('/kinect/qhd/image_color_rect', Image, queue_size=2)  # image publisher
        self.bridge = CvBridge()

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for image in self.images:
                img = cv2.imread(image)
                image_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.image_topic.publish(image_msg)
                rate.sleep()

if __name__ == '__main__':
    pubber = image_pub()
    pubber.run()
