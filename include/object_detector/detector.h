//
// Created by banerjs on 12/13/16.
//

#ifndef SAN_OBJECT_DETECTOR_DETECTOR_NODE_H
#define SAN_OBJECT_DETECTOR_DETECTOR_NODE_H

#include <string>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_service_options.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "object_detector/Object.h"
#include "object_detector/SceneQuery.h"
#include "object_detector/ImageQuery.h"

#include "detector.h"

// Weird requirement to redefine the structures here
extern "C" {
  typedef struct darknet_object darknet_object;
  typedef struct network network;
};

class Detector
{
public:
  // Constructors
  Detector(ros::NodeHandle &handle, ros::NodeHandle &private_handle)
  : nh_(handle), private_nh_(private_handle), it_(private_handle)
  {}

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
  void imageSubscriberCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
   * Callback for a scene query
   */
  bool sceneQueryCallback(object_detector::SceneQuery::Request &req,
    object_detector::SceneQuery::Response &res);

  /**
   * Callback for an image query
   */
  bool imageQueryCallback(object_detector::ImageQuery::Request &req,
    object_detector::ImageQuery::Response &res);

private:
  // Node Handles to communicate with the param server
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Publishers, Subscribers, and Service Servers
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
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

  // Darknet variables
  float probability_threshold_;
  network *net_;
  char **class_names_;
};

#endif //SAN_OBJECT_DETECTOR_DETECTOR_NODE_H
