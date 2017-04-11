#!/usr/bin/env python
# This script is designed to test the functionality of the image query

import rospy
import cv2
import glob
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rail_object_detector.srv import ImageQuery, ImageQueryRequest, ImageQueryResponse

COLORS = [(10,10,10), (100,10,10), (200,10,10), (10,100,10), (10,200,10),
(10,10,100), (10,10,200), (100,100,100), (200,100,100), (100,200,100),
(100,100,200)]

def main():
	bridge = CvBridge()

	rospack = rospkg.RosPack()
	images = glob.glob(rospack.get_path('rail_object_detector') + '/libs/darknet/data/*.jpg')
	service_proxy = rospy.ServiceProxy('/detector_node/objects_in_image', ImageQuery)

	for image in images:
		img = cv2.imread(image)
		img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
		resp = service_proxy(ImageQueryRequest(img_msg))

		# Draw and display the image
		resp_cv = bridge.imgmsg_to_cv2(resp.image, "bgr8")
		person_images = []
		for idx,obj in enumerate(resp.objects):
			# cv2.rectangle(
			# 	resp_cv,
			# 	(obj.left_bot_x, obj.left_bot_y),
			# 	(obj.right_top_x, obj.right_top_y),
			# 	COLORS[idx],
			# 	3
			# )
			if obj.label == 'person':
					width = obj.right_top_x - obj.left_bot_x
					height = obj.left_bot_y - obj.right_top_y
					wid_div_fac = 4
					hei_div_fac = 6
					start_x = max(obj.left_bot_x - width/wid_div_fac, 0)
					start_y = max(obj.right_top_y - height/hei_div_fac, 0)
					end_x = min(obj.right_top_x + width/wid_div_fac, len(resp_cv))
					end_y = min(obj.left_bot_y +height/hei_div_fac, len(resp_cv[0]))
					new_img = resp_cv[start_y:end_y, start_x:end_x]
					person_images.append(new_img)

		print "Objects in scene:", ",".join([x.label for x in resp.objects])

		# cv2.imshow('image', resp_cv)
		for index, image in enumerate(person_images):
			if len(image) < 100 or len(image[0]) < 100:
				continue
			cropped_img = bridge.cv2_to_imgmsg(image, "bgr8")
			cropped_resp = service_proxy(ImageQueryRequest(cropped_img))
			cropped_resp_im = bridge.imgmsg_to_cv2(cropped_resp.image, "bgr8")
			for idx,obj in enumerate(cropped_resp.objects):
				cv2.rectangle(
					cropped_resp_im,
					(obj.left_bot_x, obj.left_bot_y),
					(obj.right_top_x, obj.right_top_y),
					COLORS[idx],
					3
				)
			cv2.imshow('person'+str(index), cropped_resp_im)

			cv2.waitKey(5000)

if __name__ == '__main__':
	main()
