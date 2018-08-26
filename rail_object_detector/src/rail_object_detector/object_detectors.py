#!/usr/bin/env python
# This file is responsible for bridging ROS to the ObjectDetector class (built with PyCaffe)
import os
import sys
import cv2
import time

import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg

from cv_bridge import CvBridge, CvBridgeError
from rail_object_detector import drfcn_detector

from sensor_msgs.msg import Image, CompressedImage
from rail_object_detection_msgs.msg import Object, Detections

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


class DRFCNDetector():
    """
    This class interfaces to the deformable R-FCN for object detection
    """
    def __init__(self):
        self.objects = []
        self.image_datastream = None
        self.input_image = None

        self.bridge = CvBridge()

        self.debug = rospy.get_param('~debug', default=False)
        self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/qhd/image_color_rect')
        self.use_compressed_image = rospy.get_param('~use_compressed_image', default=False)

        rospack = rospkg.RosPack()
        self.model_filename = rospy.get_param(
            '~model_filename',
            os.path.join(rospack.get_path('rail_object_detector'), 'libs', 'drfcn' , 'model', 'rfcn_dcn_coco')
        )
        self.detector = drfcn_detector.Detector(self.model_filename)

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
                :param image_msg: Image data
                :return: None
                """

        header = image_msg.header
        image_cv = self._convert_msg_to_image(image_msg)
        if image_cv is None:
            return
        self.objects = self.detector.detect_objects(image_cv)
        if self.debug:
            debug_im = self._draw_boxes(image_cv, self.objects, self.detector.classes)
            try:
                image_msg = self.bridge.cv2_to_imgmsg(debug_im, "bgr8")
            except CvBridgeError as e:
                print e
            image_msg.header = header
            self.image_pub.publish(image_msg)

        # Instantiate detections object
        object_arr = Detections()
        object_arr.header = header
        for cls_idx, cls_name in enumerate(self.detector.classes):
            cls_dets = self.objects[cls_idx]
            for obj in cls_dets:
                msg = Object()
                msg.left_bot_x = int(obj[0])
                msg.right_top_y = int(obj[1])
                msg.right_top_x = int(obj[2])
                msg.left_bot_y = int(obj[3])
                msg.centroid_x = int((obj[0] + obj[2])/2)
                msg.centroid_y = int((obj[1] + obj[3])/2)
                msg.probability = obj[-1]
                msg.label = cls_name
                object_arr.objects.append(msg)
        self.object_pub.publish(object_arr)

    def _draw_boxes(self, im, dets, classes, scale=1.0):
        for cls_idx, cls_name in enumerate(classes):
            cls_dets = dets[cls_idx]
            color = np.random.randint(0, 256, 3)
            for det in cls_dets:
                bbox = det[:4] * scale
                cv2.rectangle(im, (bbox[0], bbox[1]),
                              (bbox[2], bbox[3]), color, thickness=3)

                if cls_dets.shape[1] == 5:
                    score = det[-1]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(im, '{:s} {:.3f}'.format(cls_name, score), (bbox[0], bbox[1]),
                                font, fontScale=0.75, color=(0, 0, 0), thickness=2)
        return im

    def start(self,
            pub_image_topic='~debug/object_image',
            pub_object_topic='~detections'):
        if not self.use_compressed_image:
            rospy.Subscriber(self.image_sub_topic_name, Image,
                             self._parse_image)  # subscribe to sub_image_topic and callback parse
        else:
            rospy.Subscriber(self.image_sub_topic_name + '/compressed', CompressedImage, self._parse_image)
        if self.debug:
            self.image_pub = rospy.Publisher(pub_image_topic, Image, queue_size=2)  # image publisher
        self.object_pub = rospy.Publisher(pub_object_topic, Detections, queue_size=2)  # detections publisher

if __name__ == '__main__':
    rospy.init_node('drfcn_node')
    detector = DRFCNDetector()
    detector.start()
    rospy.spin()
