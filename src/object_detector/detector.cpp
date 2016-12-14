//
// Created by banerjs on 12/13/16.
//

#include "object_detector/detector.h"

// Helper functions
object_detector::ObjectPtr createObjectMessage(darknet_object
  &detected_object);

// Functions from darknet
extern "C" bool darknet_detect(network *net, IplImage *image,
  float thresh, char **class_names, darknet_object **detected_objects, int
  *num_detected_objects);
extern "C" network create_network(char *cfg_filename, char *weight_filename);
extern "C" char **get_class_names(char *datacfg_filename);

// Implementation of start
bool Detector::start()
{
  // Reset the pointers
  latest_image_.reset();

  // Initialize the parameters
  int num_service_threads;
  std::string scene_service_name;
  std::string image_service_name;
  std::string image_sub_topic_name;
  std::string datacfg_filename;
  std::string cfg_filename;
  std::string weight_filename;

  private_nh_.param("num_service_threads", num_service_threads, int(0));
  private_nh_.param("scene_service_name", scene_service_name, std::string
    ("/san_object_detector/objects_in_scene"));
  private_nh_.param("image_service_name", image_service_name, std::string
    ("/san_object_detector/objects_in_image"));
  private_nh_.param("image_sub_topic_name", image_sub_topic_name, std::string
    ("/kinect/hd/image_color_rect"));
  private_nh_.param("probability_threshold", probability_threshold_, float(
    .25));
  private_nh_.param("datacfg_filename", datacfg_filename, std::string
    ("/home/banerjs/Libraries/SAN/object_detector/libs/darknet/cfg/coco.data"));
  private_nh_.param("cfg_filename", cfg_filename, std::string
    ("/home/banerjs/Libraries/SAN/object_detector/libs/darknet/cfg/yolo.cfg"));
  private_nh_.param("weight_filename", weight_filename, std::string(
    "/home/banerjs/Libraries/SAN/object_detector/libs/darknet/yolo.weights"));

  // Load the network into memory
  class_names_ = get_class_names((char *)datacfg_filename.c_str());
  net_ = create_network((char *)cfg_filename.c_str(), (char *)weight_filename
    .c_str());

  // FIXME: Cannot figure out the size of the class_names_ array :(
  // int num_classes = sizeof(class_names_) / sizeof(class_names_[0]);
  // ROS_INFO("Created: %d classes", num_classes);

  // Create the callback queues for the services and the subscribers
  // NOTE: Might want to add the compressed hint to this subscription
  image_sub_ = it_.subscribe(image_sub_topic_name, 1,
                             &Detector::imageSubscriberCallback, this);

  // Initialize the service callback queues
  scene_callback_q_ = boost::make_shared<ros::CallbackQueue>();
  image_callback_q_ = boost::make_shared<ros::CallbackQueue>();

  // Service for the scene query
  ros::AdvertiseServiceOptions scene_opts;
  boost::function<bool(object_detector::SceneQuery::Request&,
                       object_detector::SceneQuery::Response&)>
    scene_callback_ptr = boost::bind(&Detector::sceneQueryCallback, this,
                                     _1, _2);
  scene_opts.init(scene_service_name, scene_callback_ptr);
  scene_opts.callback_queue = scene_callback_q_.get();

  ros::AdvertiseServiceOptions image_opts;
  boost::function<bool(object_detector::ImageQuery::Request&,
                       object_detector::ImageQuery::Response&)>
    image_callback_ptr = boost::bind(&Detector::imageQueryCallback, this, _1,
                                     _2);
  image_opts.init(image_service_name, image_callback_ptr);
  image_opts.callback_queue = image_callback_q_.get();

  // Advertise the service and start the spinners
  scene_query_server_ = nh_.advertiseService(scene_opts);
  if (!scene_query_server_)
  {
    ROS_FATAL("Error in starting the scene service");
    return false;
  }

  image_query_server_ = nh_.advertiseService(image_opts);
  if (!image_query_server_)
  {
    ROS_FATAL("Error in starting the image scene service");
    return false;
  }

  scene_spinner_ = boost::make_shared<ros::AsyncSpinner>(num_service_threads,
                                                         scene_callback_q_
                                                           .get());
  scene_spinner_->start();
  image_spinner_ = boost::make_shared<ros::AsyncSpinner>(num_service_threads,
                                                         image_callback_q_
                                                           .get());
  image_spinner_->start();
  return true;
}

// Implementation of stop
bool Detector::stop()
{
  scene_spinner_->stop();
  scene_spinner_.reset();
  scene_query_server_.shutdown();
  image_spinner_->stop();
  image_spinner_.reset();
  image_query_server_.shutdown();

  image_sub_.shutdown();

  scene_callback_q_.reset();
  image_callback_q_.reset();

  latest_image_.reset();

  // FIXME: Double free when trying to free. Leave for now
  // free(net_);

  // FIXME: Cannot figure out the size of the class, so can't free :(
  // int num_classes = sizeof(class_names_) / sizeof(class_names_[0]);
  // ROS_INFO("Freeing: %d classes", num_classes);
  // for (int i = 0; i < num_classes; i++)
  // {
  //   free(class_names_[i]);
  // }
  // free(class_names_);
  return true;
}

// Implementation of the subscriber
void Detector::imageSubscriberCallback(
  const sensor_msgs::ImageConstPtr &msg)
{
  // Update the cache of the latest image if we can acquire the lock to it
  boost::mutex::scoped_lock lock(mutex_, boost::try_to_lock);

  if (lock)
  {
    latest_image_ = msg;
  }
}

// Implementation of scene query callback
bool Detector::sceneQueryCallback(object_detector::SceneQuery::Request &req,
  object_detector::SceneQuery::Response &res)
{
  cv_bridge::CvImagePtr cv_ptr;
  {
    // Try to get a lock to the latest image of the scene
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_image_.get() == NULL)
    {
      ROS_INFO("No images from camera");
      return true;
    }
    try
    {
      cv_ptr = cv_bridge::toCvCopy(latest_image_,
                                   sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &ex)
    {
      ROS_ERROR("Unable to convert image message to mat: %s", ex.what());
      return false;
    }
  }

  // Process the image of the scene
  IplImage ipl_image = (IplImage)cv_ptr->image;
  darknet_object *detected_objects;
  int num_detected_objects;
  bool detection_success = darknet_detect(&net_, &ipl_image,
                                          probability_threshold_,
                                          class_names_, &detected_objects,
                                          &num_detected_objects);
  if (!detection_success)
  {
    ROS_ERROR("There was a failure during detection");
    return false;
  }

  // Convert the data to ROS Message format
  res.image = *(cv_ptr->toImageMsg());
  res.header.frame_id = res.image.header.frame_id;
  res.header.stamp = ros::Time::now();

  for (int i = 0; i < num_detected_objects; i++)
  {
    object_detector::ObjectPtr obj_ptr = createObjectMessage
      (detected_objects[i]);
    res.objects.push_back(*obj_ptr);
  }

  // Free the resources
  free(detected_objects);

  return true;
}

// Implementation of the image query callback
bool Detector::imageQueryCallback(object_detector::ImageQuery::Request &req,
  object_detector::ImageQuery::Response &res)
{
  // Create a CV image from the image message
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception &ex)
  {
    ROS_ERROR("Unable to convert image message to mat: %s", ex.what());
    return false;
  }

  // Process the image
  IplImage ipl_image = (IplImage)cv_ptr->image;
  darknet_object *detected_objects;
  int num_detected_objects;
  bool detection_success = darknet_detect(&net_, &ipl_image,
                                          probability_threshold_,
                                          class_names_, &detected_objects,
                                          &num_detected_objects);
  if (!detection_success)
  {
    ROS_ERROR("There was a failure during detection");
    return false;
  }

  // Populate the response object
  res.image = *(cv_ptr->toImageMsg());
  res.header.frame_id = res.image.header.frame_id;
  res.header.stamp = ros::Time::now();

  for (int i = 0; i < num_detected_objects; i++)
  {
    object_detector::ObjectPtr obj_ptr = createObjectMessage
      (detected_objects[i]);
    res.objects.push_back(*obj_ptr);
  }

  // Free the resources
  free(detected_objects);

  return true;
}

// Implementation of createObjectMessage
object_detector::ObjectPtr createObjectMessage(darknet_object
  &detected_object)
{
  object_detector::ObjectPtr msg =
    boost::make_shared<object_detector::Object>();
  msg->label = std::string(detected_object.label);
  msg->probability = detected_object.probability;
  msg->centroid_x = detected_object.centroid_x;
  msg->centroid_y = detected_object.centroid_y;
  msg->left_bot_x = detected_object.left_bot_x;
  msg->left_bot_y = detected_object.left_bot_y;
  msg->right_top_x = detected_object.right_top_x;
  msg->right_top_y = detected_object.right_top_y;
  return msg;
}
