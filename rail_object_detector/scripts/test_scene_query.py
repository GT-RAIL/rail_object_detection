#!/usr/bin/env python
# This script is designed to test the functionality of the scene query

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rail_object_detection_msgs.srv import SceneQuery, SceneQueryRequest, SceneQueryResponse

COLORS = [(10,10,10), (100,10,10), (200,10,10), (10,100,10), (10,200,10),
(10,10,100), (10,10,200), (100,100,100), (200,100,100), (100,200,100),
(100,100,200)]

def main():
	rospy.init_node('test_scene_query_node')
	bridge = CvBridge()

	service_proxy = rospy.ServiceProxy('/darknet_node/objects_in_scene', SceneQuery)

	rate = rospy.Rate(.4)
	while not rospy.is_shutdown():
		resp = service_proxy()

		try:
			# Draw and display the image
			resp_cv = bridge.imgmsg_to_cv2(resp.image, "bgr8")
			for idx,obj in enumerate(resp.objects):
				cv2.rectangle(
					resp_cv,
					(obj.left_bot_x, obj.left_bot_y),
					(obj.right_top_x, obj.right_top_y),
					COLORS[idx%len(COLORS)],
					3
				)
			cv2.imshow('image', resp_cv)
			cv2.waitKey(100)
			rospy.loginfo("objects: %s" % ",".join([x.label for x in resp.objects]))
		except CvBridgeError:
			pass

		rate.sleep()

if __name__ == '__main__':
	main()
