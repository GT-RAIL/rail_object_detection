# RAIL Object Detector

[![Build Status](https://travis-ci.org/GT-RAIL/rail_object_detector.svg?branch=master)](https://travis-ci.org/GT-RAIL/rail_object_detector)

## Temporary Warning

The system will not work using the weights from the server.

## Two Minute Intro

This detector uses [darknet](https://github.com/pjreddie/darknet) to perform object detection. It provides the ability to query for objects in an image through both services as well as from a topic.

The response to all queries contains a list of objects, each of which has the following properties:

1. `label`
1. `probability` - confidence value in recognition
1. `centroid_x` - X pixel value of the centroid of the bounding box
1. `centroid_y` - Y pixel value of the centroid of the bounding box
1. `left_bot_x` - X pixel value of bottom-left corner of bounding box
1. `left_bot_y` - Y pixel value of bottom-left corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box

#### Querying through services

There are two modes of querying:

- Scene Queries
- Image Queries

Scene Queries are served by first subscribing to an existing camera sensor topic. Then at the moment of the query, we run object recognition on the latest frame from the camera and the resulting objects in that scene are returned after however long darknet takes.

Image Queries require an image to be sent alongwith the query. Object recognition is performed on this input image, and the detected objects as well as the original image are sent back.

#### Query through topic

If enabled, the detector subscribes to an existing camera sensor topic and grabs images from this camera at (prespecified) intervals. After performing object detection on the grabbed image, the detector publishes the list of objects that were found to the query topic and with the timestamp of the image which was grabbed for detection.

The interval for grabbing images is specified in the form of a frequency. If the desired frequency exceeds the maximum frequency of operation of the detector (~1 Hz on CPU), we limit to the maximum frequency of operation.

## Menu
 * [Installation](#installation)
 * [Testing your Installation](#testing-your-installation)
 * [ROS Nodes](#ros-nodes)
 * [Startup](#startup)
 * [Building with CUDA](#building-with-cuda)
 * [Scope for Improvement](#scope-for-improvement)

## Installation

1. Put this package into your workspace
1. Assuming `WS` as the top level directory of this package (where this README is located), navigate to `${WS}/libs/darknet`
1. Download the weights from a remote location (as specified by Meera) `wget http://pjreddie.com/media/files/yolo.weights`
1. Run `catkin_make` and enjoy the ability to use object detection! (If you need to update the file paths, use absolute paths!)

## Testing your Installation

Three optional test scripts are included in the `scripts` directory (`test_image_query.py`, `test_scene_query.py`, and `test_detections_topic.py`).  To test your installation, do the following:

- Copy some test .jpg images into the `libs/darknet/data` directory.
- Run a camera with your favorite ROS camera node.
- Launch the `detector_node` node with the image topic of your camera and image queries enabled:
```
roslaunch rail_object_detector detector.launch use_image_service:=true image_sub_topic_name:=[camera image here]`
```
- Run the scene query test script; this should periodically detect and recognize objects in images from your camera:
```
rosrun rail_object_detector test_scene_query.py
```
- Run the image query test script; this should run object recognition on the images you copied into `data`:
```
rosrun rail_object_detector test_image_query.py
```
- Shutdown the previous launch and restart with services disabled but the detections topic enabled (this is due to an inexplicable bug on CPU mode where the topic and services don't seem to work well together):
```
roslaunch rail_object_detector detector.launch use_image_service:=true image_sub_topic_name:=[camera image here]
```
- Run the topic test script; this should run object recognition in the backround and print to console the list of objects that were detected along with the timestamp:
```
rosrun rail_object_detector test_detections_topic.py
```

## ROS Nodes

### detector_node

Wrapper for object detection through ROS services.  Relevant services and parameters are as follows:

* **Services**
  * `detector_node/objects_in_scene` ([object_detector/SceneQuery](https://github.com/gt-rail-internal/SAN/blob/master/object_detector/srv/SceneQuery.srv))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Scene Query service: recognize objects in the latest image from the camera stream `image_sub_topic_name`.  Takes no input, and outputs a list of detected, labeled objects and a corresponding image.  Only advertised if `use_scene_service` is true.
  * `detector_node/objects_in_image` ([object_detector/ImageQuery](https://github.com/gt-rail-internal/SAN/blob/master/object_detector/srv/ImageQuery.srv))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image Query service: recognize objects in an image passed to the service.  Takes an image as input, and outputs a list of detected, labeled objects and a corresponding image. Only advertised if `use_image_service` is true.
* **Topics**
  * `detector_node/detections` ([object_detector/Detections](https://github.com/gt-rail-internal/SAN/tree/master/object_detector/msg/Detections.msg))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Topic with object detections performed in the background by grabbing images at a specified interval. Only advertised if `publish_detections_topic` is true.
* **Parameters**
  * `num_service_threads` (`int`, default: 0)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Number of asynchronous threads that can be used to service each of the services. `0` implies the use of one thread per processor
  * `use_scene_service` (`bool`, default: true)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Scene Query service
  * `use_image_service` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Image Query service
  * `publish_detections_topic` (`bool`, default: false)
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Detections topic
  * `image_sub_topic_name` (`string`, default: "/kinect/hd/image_color_rect")
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image topic name to subscribe to for the Scene Query service
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

Simply run the launch file to bring up all of the package's functionality (default: use Scene Queries only):
```
roslaunch rail_object_detector detector.launch
```

## Building with CUDA

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
