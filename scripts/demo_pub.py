#!/usr/bin/env python
import sys

import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_pub():
    def __init__(self):
        rospy.init_node("image_pub")
        self.image_in = cv2.imread('/home/asilva/Downloads/frame0000.jpg')
        self.image_topic = rospy.Publisher('/kinect/qhd/image_color_rect', Image, queue_size=2)  # image publisher
        self.bridge = CvBridge()

    def run(self):
        image_msg = self.bridge.cv2_to_imgmsg(self.image_in, "bgr8")

        self.image_topic.publish(image_msg)

if __name__ == '__main__':
    try:
        pubber = image_pub()
        print 'hi'
        while True:
            pubber.run()
    except rospy.ROSInterruptException:
        pass
