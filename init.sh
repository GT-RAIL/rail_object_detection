#!/bin/bash

mkdir -p ./libs/mxnet
mkdir -p ./libs/model

cd libs/bbox
python setup_linux.py build_ext --inplace
cd ../dataset/pycocotools
python setup_linux.py build_ext --inplace
cd ../../nms
python setup_linux.py build_ext --inplace
cd ../..
