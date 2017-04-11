#!/usr/bin/env python
# This script is designed to test the functionality of the scene query

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rail_object_detector.srv import SceneQuery, SceneQueryRequest, SceneQueryResponse

COLORS = [(10,10,10), (100,10,10), (200,10,10), (10,100,10), (10,200,10),
(10,10,100), (10,10,200), (100,100,100), (200,100,100), (100,200,100),
(100,100,200)]

def main():
	rospy.init_node('test_scene_query_node')
	bridge = CvBridge()

	service_proxy = rospy.ServiceProxy('/detector_node/objects_in_scene', SceneQuery)

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		resp = service_proxy()

		try:
			# Draw and display the image
			person_images = []
			resp_cv = bridge.imgmsg_to_cv2(resp.image, "bgr8")
			for idx,obj in enumerate(resp.objects):
				cv2.rectangle(
					resp_cv,
					(obj.left_bot_x, obj.left_bot_y),
					(obj.right_top_x, obj.right_top_y),
					COLORS[idx%len(COLORS)],
					3
				)
				if obj.label == 'person':
					width = obj.right_top_x - obj.left_bot_x
					height = obj.left_bot_y - obj.right_top_y
					print width
					print height
					start_x = max(obj.left_bot_x - width/2, 0)
					start_y = max(obj.right_top_y - height/2, 0)
					end_x = min(obj.right_top_x + width/2, len(resp_cv))
					end_y = min(obj.left_bot_y +height/2, len(resp_cv[0]))
					print start_x
					print end_x
					print start_y
					print end_y
					new_img = resp_cv[start_x:end_x, start_y:end_y]
					person_images.append([width, height, start_x, end_x, start_y, end_y])

			cv2.imshow('image', resp_cv)
			for image in person_images:
				cv2.imshow('person_image', image)
			cv2.waitKey(100)
			rospy.loginfo("objects: %s" % ",".join([x.label for x in resp.objects]))
			rospy.loginfo("people: %s" % ",".join([x for x in person_images]))
		except CvBridgeError:
			pass

		rate.sleep()

if __name__ == '__main__':
	main()
