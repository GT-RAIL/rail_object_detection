#!/usr/bin/env python
# This script is designed to test the functionality of the object detections
# topic

import rospy
from rail_object_detection_msgs.msg import Detections

def detections_callback(data):
	rospy.loginfo("*****************New Message:*********************")
	rospy.loginfo(
		"Frame@Timestamp: %s@%s" % (data.header.frame_id, data.header.stamp,)
	)
	rospy.loginfo("Objects: %s" % ",".join([x.label for x in data.objects]))

def main():
	rospy.init_node('test_detections_topic')
	rospy.Subscriber('/darknet_node/detections', Detections, detections_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
