//
// Created by banerjs on 12/13/16.
//

#ifndef RAIL_OBJECT_DETECTOR_DARKNET_DETECTOR_H
#define RAIL_OBJECT_DETECTOR_DARKNET_DETECTOR_H

#include <string>
#include <vector>
#include <time.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "rail_object_detection_msgs/Detections.h"
#include "rail_object_detection_msgs/Object.h"
#include "rail_object_detection_msgs/SceneQuery.h"
#include "rail_object_detection_msgs/ImageQuery.h"

#include "detector.h"

using namespace rail_object_detection_msgs;

namespace rail_object_detector
{

// Weird requirement to redefine the structures here
extern "C" {
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

typedef enum
{
  CONSTANT, STEP, EXP, POLY, STEPS, SIG, RANDOM
} learning_rate_policy;
typedef struct tree tree;
typedef struct layer layer;

typedef struct network
{
  float *workspace;
  int n;
  int batch;
  int *seen;
  float epoch;
  int subdivisions;
  float momentum;
  float decay;
  layer *layers;
  int outputs;
  float *output;
  learning_rate_policy policy;

  float learning_rate;
  float gamma;
  float scale;
  float power;
  int time_steps;
  int step;
  int max_batches;
  float *scales;
  int *steps;
  int num_steps;
  int burn_in;

  int adam;
  float B1;
  float B2;
  float eps;

  int inputs;
  int h, w, c;
  int max_crop;
  int min_crop;
  float angle;
  float aspect;
  float exposure;
  float saturation;
  float hue;

  int gpu_index;
  tree *hierarchy;

#ifdef GPU
  float **input_gpu;
  float **truth_gpu;
#endif
} network;
};

class DarknetDetector
{
public:
  // Constructors
  DarknetDetector(ros::NodeHandle &handle, ros::NodeHandle &private_handle)
    : nh_(handle), private_nh_(private_handle), it_(private_handle),
      perform_detections_(false), detections_thread_(NULL)
  { }

  // Methods

  /**
   * Start the detector
   */
  bool start();

  /**
   * Stop the detector
   */
  bool stop();

  /**
   * Callback for the images coming in on the image topic
   */
  void imageSubscriberCallback(const sensor_msgs::ImageConstPtr &msg);

  /**
   * Callback for a scene query
   */
  bool sceneQueryCallback(SceneQuery::Request &req, SceneQuery::Response &res);

  /**
   * Callback for an image query
   */
  bool imageQueryCallback(ImageQuery::Request &req, ImageQuery::Response &res);

  /**
   * Runnable function that performs detections in the background and
   * publishes them to the detections topic. (Callback)
   */
  void runBackgroundDetections();

  /**
   * Callback function for the background publisher which is called back on a
   * timer depending on the desired frequency of detections
   */
   void backgroundDetectionCallback(const ros::TimerEvent &e);

private:
  // Node Handles to communicate with the param server
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Booleans to track which abilities to use
  bool use_scene_service_;
  bool use_image_service_;
  bool publish_detections_topic_;

  // Parameters for the abilities, such as publish frequencies, etc.
  float max_desired_publish_freq_;

  // Publishers, Subscribers, and Service Servers
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher detections_pub_;
  ros::ServiceServer scene_query_server_;
  ros::ServiceServer image_query_server_;

  // Locking mechanisms
  boost::mutex mutex_;

  // Image Pointer for the darknet detector
  sensor_msgs::ImageConstPtr latest_image_;   // Access to this is protected

  // Spinners and Queues
  ros::CallbackQueuePtr scene_callback_q_;
  ros::CallbackQueuePtr image_callback_q_;
  boost::shared_ptr<ros::AsyncSpinner> scene_spinner_;
  boost::shared_ptr<ros::AsyncSpinner> image_spinner_;

  // Asynchronous threads
  boost::thread *detections_thread_;
  bool perform_detections_;

  // Darknet variables
  float probability_threshold_;
  network net_;
  char **class_names_;

  // Private Methods

  /**
   * Common method that calls the darknet network for the set of objects. The
   * method takes as arguments an OpenCV image pointer and a reference to a
   * std::vector that the detected objects can be inserted into. It returns a
   * status of true if there was no issue in detection; else it returns false.
   */
  bool detectObjects(cv_bridge::CvImagePtr cv_ptr, std::vector<Object>
    &detected_objects);
};

}
#endif //RAIL_OBJECT_DETECTOR_DARKNET_DETECTOR_H
