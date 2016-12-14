//
// Created by banerjs on 12/13/16.
//

#include "object_detector/detector.h"

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

  private_nh_.param("num_service_threads", num_service_threads, int(0));
  private_nh_.param("scene_service_name", scene_service_name, std::string
    ("/san_object_detector/objects_in_scene"));
  private_nh_.param("image_service_name", image_service_name, std::string
    ("/san_object_detector/objects_in_image"));
  private_nh_.param("image_sub_topic_name", image_sub_topic_name, std::string
    ("/kinect/hd/image_color_rect"));

  // Load the network into memory

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
  image_sub_.shutdown();

  scene_callback_q_.reset();
  image_callback_q_.reset();

  latest_image_.reset();
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

  IplImage ipl_image = (IplImage)cv_ptr->image;
  ROS_INFO("Image dimensions: %d x %d", ipl_image.width, ipl_image.height);
  return true;
}

// Implementation of the image query callback
bool Detector::imageQueryCallback(object_detector::ImageQuery::Request &req,
  object_detector::ImageQuery::Response &res)
{
  return true;
}
