#!/usr/bin/env python
# This file is responsible for bridging ROS to the ObjectDetector class (built with PyCaffe)

from __future__ import division

import sys

import cv2
import rospy
import numpy as np
from lib.utils.timer import Timer
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from rail_faster_rcnn.msg import Object, Detections
import time
import matplotlib.pyplot as plt

import object_detector

# Debug Helpers
FAIL_COLOR = '\033[91m'
ENDC_COLOR = '\033[0m'


def eprint(error):
    sys.stderr.write(
        FAIL_COLOR
        + type(error).__name__
        + ": "
        + error.message
        + ENDC_COLOR
    )
# End Debug Helpers


class RCNNDetector(object):
    """
    This class takes in image data and finds / annotates objects within the image
    """

    def __init__(self):
        rospy.init_node('rcnn_detector_node')
        self.objects = []
        self.keypoint_arrays = []
        self.image_datastream = None
        self.input_image = None
        self.bridge = CvBridge()
        self.detector = object_detector.RCNNDetector()
        self.debug = rospy.get_param('~debug', default=False)
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/hd/image_color_rect')
        self.use_compressed_image = rospy.get_param('~use_compressed_image', default=False)

    def _draw_bb(self, image, bounding_box, color):
        start_x = bounding_box['x']
        start_y = bounding_box['y']
        end_x = start_x + bounding_box['w']
        end_y = start_y + bounding_box['h']
        cv2.rectangle(image,
                      (start_x, start_y),
                      (end_x, end_y),
                      color=color,
                      thickness=3)
        return image

    def _convert_msg_to_image(self, image_msg):
        """
        Convert an incoming image message (compressed or otherwise) into a cv2
        image
        """
        if not self.use_compressed_image:
            try:
                image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print e
                return None
        else:
            image_np = np.fromstring(image_msg.data, np.uint8)
            image_cv = cv2.imdecode(image_np, cv2.CV_LOAD_IMAGE_COLOR)

        return image_cv

    def _parse_image(self, image_msg):
        """
        Take in an image and draw a bounding box within it
        Publishes bounding box data onwards
        :param image_msg: Image data
        :return: None
        """
        start_time = time.time()
        header = image_msg.header

        image_cv = self._convert_msg_to_image(image_msg)
        if image_cv is None:
            return

        right_objects = []
        objects_back = self.detector.find_objects(image_cv)

        for obj_list in objects_back:
            for obj in obj_list:
                left_x = obj[0]
                top_y = obj[1]
                right_x = obj[2]
                bot_y = obj[3]
                cls = obj[4]
                right_objects.append([left_x, bot_y, right_x, top_y, cls])

        self.objects = right_objects
        if self.debug:
            for obj in self.objects:
                c = (0, 255, 0)
                x1 = int(obj[0])
                y1 = int(obj[1])
                x2 = int(obj[2])
                y2 = int(obj[3])
                image_cv = self._draw_bb(image_cv, {'x': x1,
                                                    'y': y1,
                                                    'w': x2-x1,
                                                    'h': y2-y1}, c)
            try:
                image_msg = self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
            except CvBridgeError as e:
                print e

            image_msg.header = header
            print 'time to publish:', time.time() - start_time
            self.image_pub.publish(image_msg)

        # Instantiate detections object
        obj_arr = Detections()
        obj_arr.header = header
        # obj_arr.objects = self.objects
        # For each object / keypoint set found in the image:

        for bbox_obj in self.objects:
            if len(bbox_obj) < 1:
                continue
            msg = Object()
            # TODO: add probability and centroid to be consistent with Object.msg?
            msg.label = bbox_obj[4]
            msg.left_bot_x = int(bbox_obj[0])
            msg.left_bot_y = int(bbox_obj[1])
            msg.right_top_x = int(bbox_obj[2])
            msg.right_top_y = int(bbox_obj[3])
            obj_arr.objects.append(msg)

        self.object_pub.publish(obj_arr)

    def run(self,
            pub_image_topic='~debug/obj_image',
            pub_object_topic='~detections'):
        if not self.use_compressed_image:
            rospy.Subscriber(self.image_sub_topic_name, Image, self._parse_image) # subscribe to sub_image_topic and callback parse
        else:
            rospy.Subscriber(self.image_sub_topic_name+'/compressed', CompressedImage, self._parse_image)
        if self.debug:
            self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2) # image publisher
        self.object_pub = rospy.Publisher(pub_object_topic, Detections, queue_size=2) # objects publisher
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = RCNNDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
