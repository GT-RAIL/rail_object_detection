//
// Created by banerjs on 12/13/16.
//

#include "rail_object_detector/detector.h"

using namespace rail_object_detector;

// Helper functions
//inline double_t timediff_usec(                  // Provide the difference btw
//  timespec start,                               // 2 times in usec
//  timespec end
//);
ObjectPtr createObjectMessage(                  // Create Object message
  darknet_object &detected_object
);

// Functions from darknet
extern "C" bool darknet_detect(network *net, IplImage *image,
  float thresh, char **class_names, darknet_object **detected_objects, int
  *num_detected_objects);
extern "C" network create_network(char *cfg_filename, char *weight_filename);
extern "C" char **get_class_names(char *classnames_filename);

// Implementation of start
bool Detector::start()
{
  // Reset the pointers
  latest_image_.reset();

  // Initialize the parameters
  int num_service_threads;
  std::string image_sub_topic_name;
  std::string classnames_filename;
  std::string cfg_filename;
  std::string weight_filename;

  std::stringstream classnames_default;
  std::stringstream cfg_default;
  std::stringstream weight_default;
  classnames_default << ros::package::getPath("rail_object_detector")
                     << "/libs/darknet/data/coco.names";
  cfg_default << ros::package::getPath("rail_object_detector")
              << "/libs/darknet/cfg/yolo.cfg";
  weight_default << ros::package::getPath("rail_object_detector")
                 << "/libs/darknet/yolo.weights";

  private_nh_.param("num_service_threads", num_service_threads, int(0));

  private_nh_.param("use_scene_service", use_scene_service_, bool(true));
  private_nh_.param("use_image_service", use_image_service_, bool(false));
  private_nh_.param("publish_detections_topic", publish_detections_topic_,
                    bool(false));

  private_nh_.param("max_desired_publish_freq", max_desired_publish_freq_,
                    double(1.0));

  private_nh_.param("image_sub_topic_name", image_sub_topic_name, std::string
    ("/kinect/hd/image_color_rect"));

  private_nh_.param("probability_threshold", probability_threshold_, float(
    .25));

  private_nh_.param("classnames_filename", classnames_filename,
                    classnames_default.str());
  private_nh_.param("cfg_filename", cfg_filename, cfg_default.str());
  private_nh_.param("weight_filename", weight_filename, weight_default.str());

  // Load the network into memory
  class_names_ = get_class_names((char *)classnames_filename.c_str());
  net_ = create_network(
    (char *)cfg_filename.c_str(),
    (char *)weight_filename.c_str()
  );

  // FIXME: Cannot figure out the size of the class_names_ array :(
  // int num_classes = sizeof(class_names_) / sizeof(class_names_[0]);
  // ROS_INFO("Created: %d classes", num_classes);

  // Subscribe to the image topic if the scene query or publisher must be used
  if (use_scene_service_ || publish_detections_topic_)
  {
    // NOTE: Might want to add the compressed hint to this subscription
    image_sub_ = it_.subscribe(image_sub_topic_name, 1,
                               &Detector::imageSubscriberCallback, this);
  }

  // Check to see if the scene service is to be used
  if (use_scene_service_)
  {
    // Initialize the service callback queues
    scene_callback_q_ = boost::make_shared<ros::CallbackQueue>();

    // Service for the scene query
    ros::AdvertiseServiceOptions scene_opts;
    boost::function<bool(SceneQuery::Request &, SceneQuery::Response &)>
      scene_callback_ptr = boost::bind(&Detector::sceneQueryCallback, this,
                                       _1, _2);
    scene_opts.init("objects_in_scene", scene_callback_ptr);
    scene_opts.callback_queue = scene_callback_q_.get();

    // Advertise the service
    scene_query_server_ = private_nh_.advertiseService(scene_opts);
    if (!scene_query_server_)
    {
      ROS_FATAL("Error in starting the scene service");
      return false;
    }

    // Start the spinners
    scene_spinner_ = boost::make_shared<ros::AsyncSpinner>(num_service_threads,
                                                           scene_callback_q_
                                                             .get());
    scene_spinner_->start();
  }

  // Check to see if the image service is to be used
  if (use_image_service_)
  {
    // Initialize the service callback queues
    image_callback_q_ = boost::make_shared<ros::CallbackQueue>();

    // Service for the image query
    ros::AdvertiseServiceOptions image_opts;
    boost::function<bool(ImageQuery::Request &, ImageQuery::Response &)>
      image_callback_ptr = boost::bind(&Detector::imageQueryCallback, this, _1,
                                       _2);
    image_opts.init("objects_in_image", image_callback_ptr);
    image_opts.callback_queue = image_callback_q_.get();

    // Advertise the service
    image_query_server_ = private_nh_.advertiseService(image_opts);
    if (!image_query_server_)
    {
      ROS_FATAL("Error in starting the image scene service");
      return false;
    }

    // Start the spinners
    image_spinner_ = boost::make_shared<ros::AsyncSpinner>(num_service_threads,
                                                           image_callback_q_
                                                             .get());
    image_spinner_->start();
  }

  // Check to see if detections should be published asynchronously
  if (publish_detections_topic_)
  {
    perform_detections_ = true;
    detections_pub_ = private_nh_.advertise<Detections>("detections", 2);
    detections_thread_ = new boost::thread(
      &Detector::runBackgroundDetections,
      this
    );
  }

  return true;
}

// Implementation of stop
bool Detector::stop()
{
  if (use_scene_service_ || publish_detections_topic_)
  {
    image_sub_.shutdown();
  }

  if (use_scene_service_)
  {
    scene_spinner_->stop();
    scene_spinner_.reset();
    scene_query_server_.shutdown();
    scene_callback_q_.reset();
    latest_image_.reset();
  }

  if (use_image_service_)
  {
    image_spinner_->stop();
    image_spinner_.reset();
    image_query_server_.shutdown();
    image_callback_q_.reset();
  }

  if (publish_detections_topic_)
  {
    perform_detections_ = false;
    detections_thread_->join();
    delete detections_thread_;
    detections_pub_.shutdown();
  }

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
bool Detector::sceneQueryCallback(SceneQuery::Request &req,
  SceneQuery::Response &res)
{
  cv_bridge::CvImagePtr cv_ptr;
  {
    // Try to get a lock to the latest image of the scene
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_image_.get() == NULL)
    {
      ROS_INFO_ONCE("No images from camera");
      return true;
    }
    try
    {
      cv_ptr = cv_bridge::toCvCopy(latest_image_,
                                   sensor_msgs::image_encodings::RGB8);
    }
    catch (const cv_bridge::Exception &ex)
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
    ObjectPtr obj_ptr = createObjectMessage(detected_objects[i]);
    res.objects.push_back(*obj_ptr);
  }

  // Free the resources
  free(detected_objects);

  return true;
}

// Implementation of the image query callback
bool Detector::imageQueryCallback(ImageQuery::Request &req,
  ImageQuery::Response &res)
{
  // Create a CV image from the image message
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::RGB8);
  }
  catch (const cv_bridge::Exception &ex)
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
  res.image = req.image;
  res.header.frame_id = res.image.header.frame_id;
  res.header.stamp = ros::Time::now();

  for (int i = 0; i < num_detected_objects; i++)
  {
    ObjectPtr obj_ptr = createObjectMessage(detected_objects[i]);
    res.objects.push_back(*obj_ptr);
  }

  // Free the resources
  free(detected_objects);

  return true;
}

// Implementation of run object detections
void Detector::runBackgroundDetections()
{
  ros::Time time1, time2;
  ros::Duration proctime;
  ros::Duration min_desired_sleep = ros::Duration(1/max_desired_publish_freq_);

  while (perform_detections_)
  {
    cv_bridge::CvImagePtr cv_ptr;
    time1 = ros::Time::now();
    bool image_is_valid = false;
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (latest_image_.get() == NULL)
      {
        ROS_INFO_ONCE("No images from camera");
      }
      else
      {
        try
        {
          cv_ptr = cv_bridge::toCvCopy(latest_image_,
                                       sensor_msgs::image_encodings::RGB8);
          image_is_valid = true;
        }
        catch (const cv_bridge::Exception &ex)
        {
          ROS_ERROR("Unable to convert image message to mat: %s", ex.what());
        }
      }
    }

    // Loop if the image is not valid
    if (!image_is_valid)
    {
      time2 = ros::Time::now();
      if (min_desired_sleep > time2-time1)
      {
        proctime = min_desired_sleep - (time2 - time1);
        proctime.sleep();
      }
      continue;
    }

    // Process the fetched image
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
      continue;
    }

    // Create the detections message and push back the data onto it
    Detections detections_msg;
    detections_msg.header = cv_ptr->header;
    for (int i = 0; i < num_detected_objects; i++)
    {
      ObjectPtr obj_ptr = createObjectMessage(detected_objects[i]);
      detections_msg.objects.push_back(*obj_ptr);
    }

    // Publish the data on the topic
    detections_pub_.publish(detections_msg);

    // Sleep the appropriate amount
    time2 = ros::Time::now();
    if (min_desired_sleep > time2-time1)
    {
      proctime = min_desired_sleep - (time2 - time1);
      proctime.sleep();
    }
  }
}

// Implementation of createObjectMessage
ObjectPtr createObjectMessage(darknet_object &detected_object)
{
  ObjectPtr msg = boost::make_shared<Object>();
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
