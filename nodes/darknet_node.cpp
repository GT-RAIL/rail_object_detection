//
// Created by banerjs on 12/13/16.
//

#include "rail_object_detector/darknet_detector.h"

using namespace rail_object_detector;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_handle("~");
  DarknetDetector detector(nh, private_handle);
  detector.start();
  ros::spin();
  detector.stop();
  return 0;
}
