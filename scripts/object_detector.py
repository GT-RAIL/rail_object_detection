# !/usr/bin/env python

"""
Object Detector Object for ROS
"""

from lib.fast_rcnn.config import cfg
from lib.fast_rcnn.test import im_detect
from lib.fast_rcnn.nms_wrapper import nms
import numpy as np
import sys
# TODO: Make these relative paths? Or instantiate them with the class?
caffe_root = '/home/asilva/caffe/python'
lib_root = '/home/asilva/Documents/codebase/silva/rail_faster_rcnn/scripts/lib'
sys.path.append(caffe_root)
sys.path.append(lib_root)
import caffe, os, cv2


class RCNNDetector:
    def __init__(self):
        cfg.TEST.HAS_RPN = True  # Use RPN for proposals

        # TODO: Make these relative paths?
        # prototxt = '/home/andrewsilva/faster_rcnn/py-faster-rcnn/models/coco_blocks/faster_rcnn_alt_opt/faster_rcnn_test.pt'
        caffemodel = '/home/asilva/faster_rcnn/py-faster-rcnn/data/VGG16_faster_rcnn_final'

        if not os.path.isfile(caffemodel):
            raise IOError(('{:s} not found.\nDid you run ./data/script/'
                           'fetch_faster_rcnn_models.sh?').format(caffemodel))

        # TODO Set this to your list of classes. Currently VOC 2007
        self.class_list = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
        self.net = caffe.Net(prototxt, caffemodel, caffe.TEST)

        print '\n\nLoaded network {:s}'.format(caffemodel)

        # Warmup on a dummy image
        im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
        for i in xrange(1):
            _, _ = im_detect(self.net, im)

    def find_objects(self, input_image):
        caffe.set_mode_gpu()
        caffe.set_device(0)
        scores, boxes = im_detect(self.net, input_image)
        objects_detected = []
        CONF_THRESH = 0.8
        NMS_THRESH = 0.3
        for cls_ind, cls in enumerate(self.class_list[1:]):
            cls_ind += 1  # because we skipped background
            cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack((cls_boxes,
                              cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, NMS_THRESH)
            dets = dets[keep, :]
            dets = dets[dets[:, -1] >= CONF_THRESH]
            # TODO currently below line is overwriting confidence with class. instead add a column for class
            dets[:, -1] = cls
            # append data structure with format [[x1, y1, x2, y2, obj_name], ...] for all boxes
            objects_detected.append(dets)
        return objects_detected

