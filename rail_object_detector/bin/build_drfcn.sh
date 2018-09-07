#!/bin/bash
# Build the dependencies for the drfcn

set -ex

# Get the directory that this script is located in
DIR="$( cd "$( dirname "$0" )" && pwd -P )"

mkdir -p ${DIR}/../libs/drfcn/mxnet
mkdir -p ${DIR}/../libs/drfcn/model

cd ${DIR}/../libs/drfcn/bbox
python setup_linux.py build_ext --inplace
cd ../nms
python setup_linux.py build_ext --inplace
