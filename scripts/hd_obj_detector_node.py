#!/usr/bin/env python
# Crop bbox around a person (start_x - width/2:end_x + width/2, start_y - height/2:end_y + height/2) and check for objects
# Silva

import rospy
import cv2
import glob
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rail_object_detector.srv import ImageQuery, ImageQueryRequest, ImageQueryResponse
from rail_object_detector.msg import Object, Detections
from rail_person_detector.msg import People


# Debug Helpers
FAIL_COLOR = '\033[91m'
ENDC_COLOR = '\033[0m'

def eprint(error):
	sys.stderr.write(
		FAIL_COLOR
		+ type(error).__name__
		+ ": "
		+ error.message
		+ ENDC_COLOR
	)
# End Debug Helpers


class HDObjDetector(object):
	"""
	This class takes in image data and finds / annotates objects within the image around a person
	"""

	def __init__(self):
		rospy.init_node('hd_obj_detector_node')
		self.last_image = None
		self.bridge = CvBridge()
		self.image_sub_topic_name = rospy.get_param('~image_sub_topic_name', default='/kinect/hd/image_color_rect')
		self.detections_topic_name = rospy.get_param('~detections_topic_name', default='/person_detector/people')
		self.service_proxy = rospy.ServiceProxy('/darknet_detector/objects_in_image', ImageQuery)


	def _draw_bb(self, image, bounding_box, color):
		start_x = bounding_box['x']
		start_y = bounding_box['y']
		end_x = start_x + bounding_box['w']
		end_y = start_y + bounding_box['h']
		cv2.rectangle(image,
					  (start_x, start_y),
					  (end_x, end_y),
					  color=color,
					  thickness=3)
		return image

	def _save_incoming_image(self, image_msg):
		self.last_image = image_msg

	def _check_for_hd_objects(self, detections_msg):
		## TODO: This shouldn't have to check label. I should just subscribe to the people detections node which is ???
		if self.last_image is None:
			return
		if len(detections_msg.people) <= 0:
			return
		## Convert last received image into cv2
		cv_image = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")
		person_images = []
		# hideous hack because i am tired and hungry don't judge me
		person_image_offset_x = []
		person_image_offset_y = []
		## TODO make this detections object the right message type
		hd_detections = Detections()
		hd_detections.header = self.last_image.header
		# For each person, crop a bounding box
		for obj in detections_msg.people:
				obj = obj.bounding_box
				width = obj.right_top_x - obj.left_bot_x
				height = obj.left_bot_y - obj.right_top_y
				start_x = max(obj.left_bot_x - width/2, 0)
				start_y = max(obj.right_top_y - height/2, 0)
				end_x = min(obj.right_top_x + width/2, len(cv_image))
				end_y = min(obj.left_bot_y +height/2, len(cv_image[0]))
				new_img = cv_image[start_y:end_y, start_x:end_x]
				person_images.append(new_img)
				person_image_offset_x.append(start_x)
				person_image_offset_y.append(start_y)

		# For each cropped image, run the detector on the crop
		for index, image in enumerate(person_images):
			if len(image) < 100 or len(image[0]) < 100:
				continue
			cropped_img = self.bridge.cv2_to_imgmsg(image, "bgr8")
			cropped_resp = self.service_proxy(ImageQueryRequest(cropped_img))
			cropped_resp_im = self.bridge.imgmsg_to_cv2(cropped_resp.image, "bgr8")
			for idx,obj in enumerate(cropped_resp.objects):
				if obj.label.strip() != "person":
					obj.left_bot_x += person_image_offset_x[index]
					obj.right_top_y += person_image_offset_y[index]
					hd_detections.objects.append(obj)
		self.detections_pub.publish(hd_detections)

	def run(self,
			pub_detections_topic='/hd_obj_detector/objects'):
		rospy.Subscriber(self.image_sub_topic_name, Image, self._save_incoming_image) # subscribe to sub_image_topic and callback save image
		rospy.Subscriber(self.detections_topic_name, People, self._check_for_hd_objects) # subscribe to detections and callback look for hd objs
		self.detections_pub = rospy.Publisher(pub_detections_topic, Detections, queue_size=2) # detections publisher
		rospy.spin()

if __name__ == '__main__':
	try:
		detector = HDObjDetector()
		detector.run()
	except rospy.ROSInterruptException:
		pass
