# Object Detector

## Getting Started

1. Put this package into your workspace
1. Assuming `WS` as the top level directory of this package (where this README is located), navigate to `${WS}/libs/darknet`
1. Download the weights from a remote location (as specified by Meera) `wget http://pjreddie.com/media/files/yolo.weights`
1. Update the `names` entry in `${WS}/libs/darknet/data/coco.data` to contain the absolute filesystem path to the `coco.names` file (this file lives in the directory `${WS}/libs/darknet/data/`)
1. Run `catkin_make` and enjoy the ability to use object detection! (If you need to update the file paths, use absolute paths!)

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

## Explanation of Arguments

`num_service_threads` - Number of asynchronous threads that can be used to service each of the services

`scene/image_service_name` - Name of the service that is returned when you perform a `rosservice list`

`use_scene/image_service` - Enable or Disable each of the services

`image_sub_topic_name` - The topic name of the image feed to subscribe to. Information from this feed is used by the `scene_service`

`probability_threshold` - Confidence value in recognition below which a detected object is treated as unrecognized

`datacfg/cfg/weight_filename` - Configuration files for darknet, but I have no clue what each file actually affects. Simply remember to use absolute paths for values here.

## Scope for Improvement

1. Include the ability to download the weights files automatically from the `CMakeLists.txt` file
1. Remove the reliance on absolute file paths in the C code
1. There is plenty of room for better logging - I do most of mine using the debugger, so there aren't as many status print commands as normal
1. There is a distinct lack of defensive programming against malicious (NULL) messages and the like. Beware.
1. There are most probably some memory leaks that might accumulate over a long period of time. These should be fixed at some point.
