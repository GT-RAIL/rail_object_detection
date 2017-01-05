//
// Created by banerjs on 12/11/16.
//

#ifndef SAN_OBJECT_DETECTOR_DETECTOR_H
#define SAN_OBJECT_DETECTOR_DETECTOR_H

#ifdef OPENCV

#define __USE_C99_MATH
#include <stdbool.h>

#include "network.h"
#include "opencv2/core/types_c.h"

/**
 * Structure defining the data type of the detected objects from darknet
 */
typedef struct darknet_object
{
  char *label;
  float probability;
  unsigned short centroid_x;
  unsigned short centroid_y;
  unsigned short left_bot_x;
  unsigned short left_bot_y;
  unsigned short right_top_x;
  unsigned short right_top_y;
} darknet_object;

/**
 * Given an OPENCV image, this function returns
 */
bool darknet_detect(network *net, IplImage *ipl, float thresh,
  char **class_names, darknet_object **detected_objects, int
  *num_detected_objects);

/**
 * Create the network
 */
network create_network(char *cfg_filename, char *weight_filename);

/**
 * Get the classes that have been trained so far
 */
char **get_class_names(char *classnames_filename);

#endif

#endif //SAN_OBJECT_DETECTOR_DETECTOR_H
