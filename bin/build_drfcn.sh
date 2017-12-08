#!/bin/bash
# Build the dependencies for the drfcn

set -ex

# Get the directory that this script is located in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

mkdir -p ${DIR}/../libs/mxnet
mkdir -p ${DIR}/../libs/model

cd ${DIR}/../libs/bbox
python setup_linux.py build_ext --inplace
cd ../nms
python setup_linux.py build_ext --inplace
