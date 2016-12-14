//
// Created by banerjs on 12/13/16.
//

#include "object_detector/detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_handle("~");
  Detector detector(nh, private_handle);
  detector.start();
  ros::Rate r(0.2);
//  ros::spin();
  r.sleep();
  detector.stop();
  return 0;
}