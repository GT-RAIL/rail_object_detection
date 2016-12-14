//
// Created by banerjs on 12/11/16.
//

#ifndef SAN_OBJECT_DETECTOR_DETECTOR_H
#define SAN_OBJECT_DETECTOR_DETECTOR_H

#ifdef OPENCV

#include "network.h"
#include "opencv2/core/types_c.h"

/**
 * Structure defining the data type of the detected objects from darknet
 */
typedef struct darknet_object
{
  char *label;
  float probability;
  short centroid_x;
  short centroid_y;
  short left_bot_x;
  short left_bot_y;
  short right_top_x;
  short right_top_y;
} darknet_object;

/**
 * Given an OPENCV image, this function returns
 */
darknet_object *darknet_detect(network *net, IplImage *ipl, float thresh,
  char **class_names);

/**
 * Create the network
 */
network *create_network(char *cfg_filename, char *weight_filename);

/**
 * Get the classes that have been trained so far
 */
char **get_class_names(char *datacfg_filename);

#endif

#endif //SAN_OBJECT_DETECTOR_DETECTOR_H
