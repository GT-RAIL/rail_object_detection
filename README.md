# Object Detector

## Two Minute Intro

The detector operates through services - you query for objects, and the detector responds. There are two modes of querying:

- Scene Queries
- Image Queries

Scene Queries are served by first subscribing to an existing camera sensor topic. Then at the moment of the query, we run object recognition on the latest frame from the camera and the resulting objects in that scene are returned after however long darknet takes.

Image Queries require an image to be sent alongwith the query. Object recognition is performed on this input image, and the detected objects as well as the original image are sent back.

The response to both these queries contains a list of objects, each of which has the following properties:

1. `label`
1. `probability` - confidence value in recognition
1. `centroid_x` - X pixel value of the centroid of the bounding box
1. `centroid_y` - Y pixel value of the centroid of the bounding box
1. `left_bot_x` - X pixel value of bottom-left corner of bounding box
1. `left_bot_y` - Y pixel value of bottom-left corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box
1. `right_top_x` - X pixel value of top-right corner of bounding box

## Menu
 * [Installation](#installation)
 * [Testing your Installation](#testing-your-installation)
 * [ROS Nodes](#ros-nodes)
 * [Startup](#startup)
 * [Scope for Improvement](#scope-for-improvement)

## Installation

1. Put this package into your workspace
1. Assuming `WS` as the top level directory of this package (where this README is located), navigate to `${WS}/libs/darknet`
1. Download the weights from a remote location (as specified by Meera) `wget http://pjreddie.com/media/files/yolo.weights`
1. Update the `names` entry in `${WS}/libs/darknet/cfg/coco.data` to contain the absolute filesystem path to the `coco.names` file (this file lives in the directory `${WS}/libs/darknet/data/`)
1. Run `catkin_make` and enjoy the ability to use object detection! (If you need to update the file paths, use absolute paths!)

## Testing your Installation

Two optional test scripts are included in the `scripts` directory (`test_image_query.py` and `test_scene_query.py`).  To test your installation, do the following:

1. Copy some test .jpg images into the `libs/darknet/data` directory.
1. Run a camera with your favorite ROS camera node.
1. Launch the `object_detector` node with the image topic of your camera and image queries enabled:  
 ```
 roslaunch object_detector detector.launch use_image_service:=true image_sub_topic_name:=[camera image here]
 ```
1. Run the scene query test script; this should periodically detect and recognize objects in images from your camera:  
 ```
 rosrun object_detector test_scene_query.py
 ```
1. Run the image query test script; this should run object recognition on the images you copied into `data`:  
 ```
 rosrun object_detector test_scene_query.py
 ```

## ROS Nodes

### detector_node
Wrapper for object detection through ROS services.  Relevant services and parameters are as follows:
 * **Services**
  * `detector_node/objects_in_scene` ([object_detector/SceneQuery](https://github.com/gt-rail-internal/SAN/blob/master/object_detector/srv/SceneQuery.srv))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Scene Query service: recognize objects in the latest image from the camera stream `image_sub_topic_name`.  Takes no input, and outputs a list of detected, labeled objects and a corresponding image.  Only advertised if `use_scene_service` is true.
  * `detector_node/objects_in_image` ([object_detector/ImageQuery](https://github.com/gt-rail-internal/SAN/blob/master/object_detector/srv/ImageQuery.srv))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image Query service: recognize objects in an image passed to the service.  Takes an image as input, and outputs a list of detected, labeled objects and a corresponding image. Only advertised if `use_image_service` is true.
 * **Parameters**
  * `num_service_threads` (`int`, default: 0)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Number of asynchronous threads that can be used to service each of the services
  * `use_scene_service` (`bool`, default: true)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Scene Query service
  * `use_image_service` (`bool`, default: false)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Enable or disable Image Query service
  * `image_sub_topic_name` (`string`, default: "/kinect/hd/image_color_rect")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Image topic name to subscribe to for the Scene Query service
  * `probability_threshold` (`float`, default: 0.25)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Confidence value in recognition below which a detected object is treated as unrecognized
  * `datacfg_filename` (`string`, default: "${WS}/libs/darknet/cfg/coco.data")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.
  * `cfg_filename` (`string`, default: "${WS}/libs/darknet/cfg/yolo.cfg")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.
  * `weight_filename` (`string`, default: "${WS}/libs/darknet/yolo.weights")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Configuration file for darknet.  Make sure to use an absolute path.  See darknet for details on configuration file itself.

## Startup

Simply run the launch file to bring up all of the package's functionality:
```
roslaunch object_detector detector.launch
```

## Scope for Improvement

1. Include the ability to download the weights files automatically from the `CMakeLists.txt` file
1. Remove the reliance on absolute file paths in the C code
1. There is plenty of room for better logging - I do most of mine using the debugger, so there aren't as many status print commands as normal
1. There is a distinct lack of defensive programming against malicious (NULL) messages and the like. Beware.
1. There are most probably some memory leaks that might accumulate over a long period of time. These should be fixed at some point.
