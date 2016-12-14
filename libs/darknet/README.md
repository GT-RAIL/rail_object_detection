#Darknet#
Darknet is an open source neural network framework written in C and CUDA. This is modified from Meera Hahn's modifications so that it can work with ROS. It is fast, easy to install, and supports CPU and GPU computation.

For more information see the [Darknet project website](http://pjreddie.com/darknet).

For questions or issues please use the [Google Group](https://groups.google.com/forum/#!forum/darknet).

## Modifications for ROS

The code in this repo returns an array of pointers to `darknet_object` types post detection. To prevent memory leaks, the caller must free this object post usage.

The function that should be used is `darknet_detect`: it performs the same functions as the `test_detector` function, but the API is different. Specifically, `datacfg`, `cfgfile`, and `weightfile` are hardcoded, while the image is passed in as an OPENCV image as opposed to the `filename`. The `thresh` parameter is unchanged.

This API is defined in `detector.h` and the function definition is in `detector.c`.

## Steps to use (unmodified) Darknet

### Install
```
git clone https://github.com/meera1hahn/darknet.git
cd darknet
wget http://pjreddie.com/media/files/yolo.weights
make
```


**NOTE: if you have a GPU and OPENCV before you `make` you should go to the MakeFile and change the first lines to 
`GPU=1` and `OPENCV=1`

### To run object detection on an image (uses coco classes)
`./darknet detect data/dog.jpg`

replace the dog image with your own image

`./darknet detect data/dog.jpg -thresh .2`

replace the thresh with your own threshold of confidence between 0 and 1 to only output objects detected with confidence over this threshold (default is .5)

`./darknet detect`

will prompt you to enter your image path and will continue running more images till you quit


####OUTPUT OF THE DETECTOR:

for each object the detector will output the following in the order listed:  
-class label   
-confidence (0 - 1)   
-x coordinate of center of bounding box (short)  
-y coordinate of center of bounding box (short)  
-x,y coordinate of the bottom left of the bounding box (short, short)  
-x,y coordinate of the top right of the bounding box (short, short)  



for the sample image data/dog.jpg the output will be:  
car  
0.542268  
564  
125  
456 , 174  
673 , 77  
bicycle  
0.509336  
328  
281  
99 , 439  
556 , 124  
dog    
0.558766  
218  
375  
116 , 540  
320 , 211  




### To run object detection on a video (uses coco classes)
`./darknet detector demo cfg/coco.data cfg/yolo.cfg yolo.weights <video file>`


