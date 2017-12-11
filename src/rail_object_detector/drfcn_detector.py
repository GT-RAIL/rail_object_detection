# --------------------------------------------------------
# Deformable Convolutional Networks
# Copyright (c) 2017 Microsoft
# Licensed under The Apache-2.0 License [see LICENSE for details]
# Written by Yi Li, Haocheng Zhang
# --------------------------------------------------------
import os
import sys
import numpy as np
import argparse
import logging
import pprint
import cv2

import rospkg

# TODO: When the code in setup.py starts working, then make sure to remove this
# paths hack as well
def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)
rospack = rospkg.RosPack()
LIB_PATH = os.path.join(rospack.get_path('rail_object_detector'), 'libs', 'drfcn')
add_path(LIB_PATH)

# Start importing mxnet
from utils.image import resize, transform
from cfg.drfcn_config import config, update_config

# get config
os.environ['PYTHONUNBUFFERED'] = '1'
os.environ['MXNET_CUDNN_AUTOTUNE_DEFAULT'] = '0'
os.environ['MXNET_ENABLE_GPU_P2P'] = '0'
update_config(os.path.join(LIB_PATH, 'cfg', 'rfcn_coco_demo.yaml'))

# actually import mxnet
import mxnet as mx
from core.tester import im_detect, Predictor
from symbols import *
from utils.load_model import load_param
from utils.show_boxes import show_boxes
from utils.tictoc import tic, toc
from nms.nms import py_nms_wrapper, cpu_nms_wrapper, gpu_nms_wrapper


class Detector:
    def __init__(
        self,
        model_path=os.path.join(LIB_PATH, 'model', 'rfcn_dcn_coco'),
        hd_images=False
    ):
        # IF we are guaranteed QHD / HD, we can remove scaling and resizing and be a bit faster
        # if hd_images:
        #     config.SCALES = [(1080, 1820)]
        # else:
        #     config.SCALES = [(540, 960)]
        pprint.pprint(config)
        config.symbol = 'resnet_v1_101_rfcn_dcn'
        sym_instance = eval(config.symbol + '.' + config.symbol)()
        sym = sym_instance.get_symbol(config, is_train=False)

        # set up class names
        self.num_classes = 81
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
                   'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
                   'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                   'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
                   'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
                   'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
                   'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
                   'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book',
                   'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']

        # get predictor
        self.data_names = ['data', 'im_info']
        self.label_names = []
        data = []
        sam = np.zeros((config.SCALES[0][0], config.SCALES[0][1], 3))
        im, im_scale = resize(sam, config.SCALES[0][0], config.SCALES[0][1], stride=config.network.IMAGE_STRIDE)
        im_tensor = transform(im, config.network.PIXEL_MEANS)
        im_info = np.array([[im_tensor.shape[2], im_tensor.shape[3], im_scale]], dtype=np.float32)
        data.append({'data': im_tensor, 'im_info': im_info})
        data = [[mx.nd.array(data[i][name]) for name in self.data_names] for i in xrange(len(data))]

        max_data_shape = [[('data', (1, 3, max([v[0] for v in config.SCALES]), max([v[1] for v in config.SCALES])))]]
        provide_data = [[(k, v.shape) for k, v in zip(self.data_names, data[i])] for i in xrange(len(data))]
        provide_label = [None for i in xrange(len(data))]
        arg_params, aux_params = load_param(model_path, 0, process=True)
        self.predictor = Predictor(sym, self.data_names, self.label_names,
                                   context=[mx.gpu(0)], max_data_shapes=max_data_shape,
                                   provide_data=provide_data, provide_label=provide_label,
                                   arg_params=arg_params, aux_params=aux_params)
        self.nms = gpu_nms_wrapper(config.TEST.NMS, 0)

    def detect_objects(self, input_image):
        data = []
        target_size = config.SCALES[0][0]
        max_size = config.SCALES[0][1]
        im, im_scale = resize(input_image, target_size, max_size, stride=config.network.IMAGE_STRIDE)
        im_tensor = transform(input_image, config.network.PIXEL_MEANS)
        im_info = np.array([[im_tensor.shape[2], im_tensor.shape[3], im_scale]], dtype=np.float32)
        data.append({'data': im_tensor, 'im_info': im_info})
        data = [[mx.nd.array(data[i][name]) for name in self.data_names] for i in xrange(len(data))]

        data_batch = mx.io.DataBatch(data=[data[0]], label=[], pad=0, index=0,
                                     provide_data=[[(k, v.shape) for k, v in zip(self.data_names, data[0])]],
                                     provide_label=[None])
        scales = [im_scale]
        scores, boxes, data_dict = im_detect(self.predictor, data_batch, self.data_names, scales, config)
        boxes = boxes[0].astype('f')
        scores = scores[0].astype('f')
        dets_nms = []
        for j in range(1, scores.shape[1]):
            cls_scores = scores[:, j, np.newaxis]
            cls_boxes = boxes[:, 4:8] if config.CLASS_AGNOSTIC else boxes[:, j * 4:(j + 1) * 4]
            cls_dets = np.hstack((cls_boxes, cls_scores))
            keep = self.nms(cls_dets)
            cls_dets = cls_dets[keep, :]
            cls_dets = cls_dets[cls_dets[:, -1] > 0.7, :]
            cls_dets[:4] *= im_scale
            dets_nms.append(cls_dets)
        # im = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)
        # show_boxes(im, dets_nms, self.classes, 1)
        return dets_nms

