#!/usr/bin/env python
# This script is designed to test the functionality of the image query

import rospy
import cv2
import glob
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rail_object_detection_msgs.srv import ImageQuery, ImageQueryRequest, ImageQueryResponse

COLORS = [(10,10,10), (100,10,10), (200,10,10), (10,100,10), (10,200,10),
(10,10,100), (10,10,200), (100,100,100), (200,100,100), (100,200,100),
(100,100,200)]

def main():
	bridge = CvBridge()

	rospack = rospkg.RosPack()
	images = glob.glob(rospack.get_path('rail_object_detector') + '/libs/darknet/data/*.jpg')
	service_proxy = rospy.ServiceProxy('/darknet_node/objects_in_image', ImageQuery)

	for image in images:
		img = cv2.imread(image)
		img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
		resp = service_proxy(ImageQueryRequest(img_msg))

		# Draw and display the image
		resp_cv = bridge.imgmsg_to_cv2(resp.image, "bgr8")
		for idx,obj in enumerate(resp.objects):
			cv2.rectangle(
				resp_cv,
				(obj.left_bot_x, obj.left_bot_y),
				(obj.right_top_x, obj.right_top_y),
				COLORS[idx],
				3
			)
		print "Objects in scene:", ",".join([x.label for x in resp.objects])

		cv2.imshow('image', resp_cv)
		cv2.waitKey(5000)

if __name__ == '__main__':
	main()
