#object recognition code
#meera hahn
from scipy.misc import imread, imresize, imsave
import os, sys
import subprocess
# def rec_obj(image_path, image_folder):
# 	img = imread(image_path)
# 	y_size = img.shape[0]
# 	x_size = img.shape[1]
# 	window_size = 600
# 	i = 0
# 	while(i < (y_size - window_size)):
# 		j = 0
# 		while(j < (x_size - window_size)):
# 			bin = img[i:(i+window_size), j:(j+window_size),:]
# 			#save bin as new_path
# 			imsave(image_folder + "bin.jpg", bin)
# 			#cmnd = './darknet coco test cfg/yolo-coco.cfg yolo-coco.weights ' + new_path
# 			#os.system(cmnd)
# 			j  = j + 256
# 		i = i + 256


			#parse output_file
			#get predictions
			#translate predictions to whole image_path
	#get rid of overlaping predictions
	#return the predictions

def rec_obj(image_path, image_folder):
	img = imread(image_path)
	y_size = img.shape[0]/2
	x_size = img.shape[1]/2
	window_size = 600

	for i in range(0,2):
			bin = img[:, i*x_size:(x_size + (i*x_size)),:]
			imsave(image_folder + "bin.jpg", bin)


if __name__ == '__main__':
	#path where pics will be
	path = '/home/meerahahn/Documents/classes/Mini-Projects/obj_rec/robot_things/'
	#sample image
 	rec_obj('/home/meerahahn/Documents/classes/Mini-Projects/obj_rec/robot_things/frame0001.jpg', path)

