# RAIL Object Detector

[![Build Status](https://travis-ci.org/GT-RAIL/rail_object_detector.svg?branch=master)](https://travis-ci.org/GT-RAIL/rail_object_detector)

## Two Minute Intro

This package now includes two object detectors which you may choose between, YOLOv2 and Deformable R-FCN (DRFCN). Detections from YOLOv2 are a bit faster, >10fps compared to ~4fps (on a Titan X), but less accurate than the from DRFCN.

The YOLOv2 detector uses [darknet](https://github.com/pjreddie/darknet) to perform object detection. It provides the ability to query for objects in an image through both services as well as from a topic.

The DRFCN detector is built on MXNet, and provides the ability to query for objects from a topic.

The response to all queries contains a list of objects, each of which has the following properties:

1. `label`
1. `probability` - confidence value in recognition
1. `centroid_x` - X pixel value of the centroid of the bounding box
1. `centroid_y` - Y pixel value of the centroid of the bounding box
1. `left_bot_x` - X pixel value of bottom-left corner of bounding box
1. `left_bot_y` - Y pixel value of bottom-left corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box

#### Querying through services (Darknet Only)

There are two modes of querying:

- Scene Queries
- Image Queries

Scene Queries are served by first subscribing to an existing camera sensor topic. Then at the moment of the query, we run object recognition on the latest frame from the camera and the resulting objects in that scene are returned after however long darknet takes.

Image Queries require an image to be sent alongwith the query. Object recognition is performed on this input image, and the detected objects as well as the original image are sent back.

#### Query through topic (Darknet and DRFCN)

If enabled, the detector subscribes to an existing camera sensor topic and grabs images from this camera at (prespecified) intervals. After performing object detection on the grabbed image, the detector publishes the list of objects that were found to the query topic and with the timestamp of the image which was grabbed for detection.

The interval for grabbing images is specified in the form of a frequency. If the desired frequency exceeds the maximum frequency of operation of the detector (~1 Hz on CPU), we limit to the maximum frequency of operation.

## Menu
 * [Installation](#installation)
 * [Testing your Installation](#testing-your-installation)
 * [ROS Nodes](#ros-nodes)
 * [Startup](#startup)
 * [GPU Mode](#gpu-mode)
 * [Scope for Improvement](#scope-for-improvement)

## Installation

#### Darknet Installation:

1. Put this package into your workspace
1. Assuming `WS` as the top level directory of this package (where this README is located), navigate to `${WS}/libs/darknet`
1. Download the weights into this directory from [this Google Drive link](https://drive.google.com/file/d/0B4jFwTFPtfejVUloYjU2LUpKUVk/view?usp=sharing)
1. Run `catkin_make` and enjoy the ability to use object detection! (If you need to update the file paths, use absolute paths!)

#### DRFCN Installation

Below is a list of python libraries that are required for the Deformable RFCN network in MXNet to work:
 * Cython
 * EasyDict
 * mxnet-cu80==0.12.0b20171027
 * Pillow
 * pyyaml

Note that the version of MXNet is the same as in the Deformable ConvNets repo. You may change the version if your setup requires something different, but it may not work properly.

There is a `requirements.txt` file included in the repo, which lists the above libraries. You can install all of them at once by running:
```
pip install -r requirements.txt
```

Once you have all of the requirements, installation procedes as follows:

1. Put this package into your workspace
1. Navigate to the top level of this package (where this README is located)
1. run `sh bin/build_drfcn.sh`
1. Download the model parameters from [this Dropbox link](https://www.dropbox.com/s/b5n215zidzqaxft/rfcn_dcn_coco-0000.params?dl=0) and move them into the `libs/model` subdirectory of this package.
1. Run `catkin_make` and enjoy the ability to use object detection! (If you need to update the file paths, use absolute paths!)


## Testing your Installation

Testing either detector is possible by first adding some testing .jpg images to the `libs/darknet/data` directory. Then follow the procedures below according to the detector you wish to test.

#### Darknet Testing

Three optional test scripts are included in the `scripts` directory (`test_image_query.py`, `test_scene_query.py`, and `test_detections_topic.py`).  To test your installation, do the following:

- Run a camera with your favorite ROS camera node.
- Launch the `detector_node` node with the image topic of your camera and image queries enabled:
```
roslaunch rail_object_detector darknet.launch use_image_service:=true image_sub_topic_name:=[camera image here]`
```
- Run the scene query test script; this should periodically detect and recognize objects in images from your camera:
```
rosrun rail_object_detector test_scene_query.py
```
- Run the image query test script; this should run object recognition on the images you copied into `data`:
```
rosrun rail_object_detector test_image_query.py
```
- Shutdown the previous launch and restart with services disabled but the detections topic enabled (this is due to a bug on CPU mode where the topic and services don't seem to work well together):
```
roslaunch rail_object_detector darknet.launch use_image_service:=true image_sub_topic_name:=[camera image here]
```
- Run the topic test script; this should run object recognition in the backround and print to console the list of objects that were detected along with the timestamp:
```
rosrun rail_object_detector test_detections_topic.py
```


#### DRFCN Testing

Launch the demo node like so:
```
roslaunch rail_object_detector drfcn_demo.launch
```
In a separate terminal, run:
```
rosrun image_view image_view image:=/rail_detector_node/debug/object_image
```
and you will see the image you pointed to with detected objects highlighted and labeled. It should look something like this:

![Visualization of the object detector](doc/objects.gif)

Colors change with each new detection of the object, and note that there is no tracking or propagation of labels (as on the couch in the gif above).

You can also launch the demo node with a publish rate parameter, which slows or speeds up the cycling of images in the `libs/darknet/data` directory. For example, to publish at 0.5 Hz I could run:
```
roslaunch rail_object_detector drfcn_demo.launch rate:=0.5
```

Otherwise, you can run a camera with whichever ROS camera node you would like.

1. Run a camera with your favorite ROS camera node.
1. Launch the `detector_node` node with the image topic of your camera and debug mode enabled:
```
roslaunch rail_object_detector drfcn.launch image_sub_topic_name:=[camera topic here] debug:=true`
```
Once again, in a separate terminal, run:
```
rosrun image_view image_view image:=/rail_object_detector/debug/object_image
```
and you will see the image you pointed to with detected objects highlighted and labeled.

## ROS Nodes

### detector_node

Wrapper for object detection through ROS services.  Relevant services and parameters are as follows:

* **Services** (Darknet Only)
  * `detector_node/objects_in_scene` ([object_detector/SceneQuery](srv/SceneQuery.srv))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Scene Query service: recognize objects in the latest image from the camera stream `image_sub_topic_name`.  Takes no input, and outputs a list of detected, labeled objects and a corresponding image.  Only advertised if `use_scene_service` is true.
  * `detector_node/objects_in_image` ([object_detector/ImageQuery](srv/ImageQuery.srv))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image Query service: recognize objects in an image passed to the service.  Takes an image as input, and outputs a list of detected, labeled objects and a corresponding image. Only advertised if `use_image_service` is true.
* **Topics** (Darknet and DRFCN)
  * `detector_node/detections` ([object_detector/Detections](msg/Detections.msg))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Topic with object detections performed in the background by grabbing images at a specified interval. For Darknet, advertised if `publish_detections_topic` is true. For DRFCN, this is always published.
  * `detector_node/debug/object_image` (DRFCN Only)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Topic with object detections visualized on incoming images as they come in from the subscriber. Only published if `debug:=true`. Currently unavailable for the Darknet detector.
* **Parameters** (Darknet and DRFCN)
  * `image_sub_topic_name` (`string`, default: "/kinect/qhd/image_color_rect")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image topic name to subscribe to for object detection
* **Parameters** (DRFCN Only)
  * `debug` (`bool`, default: false) (DRFCN Only)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable debug mode, which publishes input images with object bounding boxes and labels overlaid
  * `use_compressed_image` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Flag to use the compressed version of your input image stream. It will append "\_compressed" to the name of your image topic, and changes how the input images are read (should just work though)
* **Parameters** (Darknet Only)
  * `num_service_threads` (`int`, default: 0)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Number of asynchronous threads that can be used to service each of the services. `0` implies the use of one thread per processor
  * `use_scene_service` (`bool`, default: true)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Scene Query service
  * `use_image_service` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Image Query service
  * `publish_detections_topic` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Detections topic
  * `max_desired_publish_freq` (`float`, default: 1.0)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Desired frequency of object detection. If frequency exceeds maximum detector frequency, the desired value will not be achieved
  * `probability_threshold` (`float`, default: 0.25)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Confidence value in recognition below which a detected object is treated as unrecognized
  * `classnames_filename` (`string`, default: "${WS}/libs/darknet/data/coco.names")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.
  * `cfg_filename` (`string`, default: "${WS}/libs/darknet/cfg/yolo.cfg")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.
  * `weight_filename` (`string`, default: "${WS}/libs/darknet/yolo.weights")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.

## Startup

Simply run the launch file to bring up all of the package's functionality:
```
roslaunch rail_object_detector detector.launch
```
Where 'detector' is either 'darknet' or 'drfcn'. Note that the default Darknet uses scene queries only, and the default DRFCN uses image topics only.

## GPU Mode

DRFCN requires an NVIDIA GPU with at least 4GB memory, and requires no additional setup to run in GPU mode (as it is the only mode). Building Darknet for GPU mode will greatly increase detection speed, but requires some additional setup, which we detail below.

#### Building with CUDA

Darknet can be built with CUDA support to provide &gt;10x speedup in object detection. The compilation flags `DARKNET_GPU` and `DARKNET_GPU_ARCH` can be used to enable this.

**Make sure you have cleaned out any previous darknet builds by running `make clean` in the directory `rail_object_detector/libs/darknet` before attempting to build with CUDA support**

```
catkin_make -DDARKNET_GPU=1 -DDARKNET_GPU_ARCH=compute_52
```

Explanation of flags:

1. `DARKNET_GPU`: Set to 1 in order to enable GPU. Any other value disables it
1. `DARKNET_GPU_ARCH`: Set to the compute capability of your CUDA enabled GPU. You can look this up on [Wikipedia](https://en.wikipedia.org/wiki/CUDA#GPUs_supported)

## Scope for Improvement

1. Scene Query and Publishing the topic don't seem to work well together for some unfathomable reason.
1. Cleaning build artifacts `catkin_make clean` does not cleanup darknet build artifacts.
1. Include the ability to download the weights files automatically from the `CMakeLists.txt` file
1. There is plenty of room for better logging - I do most of mine using the debugger, so there aren't as many status print commands as normal
1. There is a distinct lack of defensive programming against malicious (NULL) messages and the like. Beware.
1. There are most probably some memory leaks that might accumulate over a long period of time. These should be fixed at some point.
