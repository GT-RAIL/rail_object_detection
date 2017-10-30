# --------------------------------------------------------
# Fast R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

# from fast_rcnn.config import cfg
# from nms.gpu_nms import gpu_nms
# from nms.cpu_nms import cpu_nms
#
# def nms(dets, thresh, force_cpu=False):
#     """Dispatch to either CPU or GPU NMS implementations."""
#
#     if dets.shape[0] == 0:
#         return []
#     if cfg.USE_GPU_NMS and not force_cpu:
#         return gpu_nms(dets, thresh, device_id=cfg.GPU_ID)
#     else:
#         return cpu_nms(dets, thresh)

# # --------------------------------------------------------
# # Fast R-CNN
# # Copyright (c) 2015 Microsoft
# # Licensed under The MIT License [see LICENSE for details]
# # Written by Ross Girshick
# # --------------------------------------------------------
#
from config import cfg
from py_cpu_nms import py_cpu_nms
# from cpu_nms import cpu_nms
# from gpu_nms import gpu_nms


def nms(dets, thresh, force_cpu=False):
    """Dispatch to either CPU or GPU NMS implementations."""

    if dets.shape[0] == 0:
        return []
    # if cfg.USE_GPU_NMS and not force_cpu:
    #     return gpu_nms(dets, thresh, device_id=cfg.GPU_ID)
    # else:
    return py_cpu_nms(dets, thresh)
