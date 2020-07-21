#!/usr/bin/env python

import rospy
from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import time
import cv2
import re
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
import numpy as np

def qrcode_localization():

	rospy.init_node('qrcode_localization')
	pub_camera_pose = rospy.Publisher('usbcamera_pose', PoseWithCovarianceStamped, queue_size=10)
 
	# initialize the video stream and allow the camera sensor to warm up
	print("[INFO] starting video stream...")
	# vs = VideoStream(src=0).start()
	vs = VideoStream(src=2).start()
	#time.sleep(2)

	# loop over the frames from the video stream
	while True:
		# grab the frame from the threaded video stream and resize it to
		# have a maximum width of 600 pixels
		frame = vs.read()
		frame = imutils.resize(frame, width=600)
		
		size = frame.shape
		image_center_x = size[1]/2
		image_center_y = size[0]/2
		image_center = (image_center_x, image_center_y)
	
		cv2.circle(frame, image_center, 5, (0, 255, 255), -1)
		cv2.putText(frame, "C", image_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

		# find the qrcodes in the frame and decode each of the qrcodes
		qrcodes = pyzbar.decode(frame)

		# loop over the detected qrcodes
		for qrcode in qrcodes:
			# extract the bounding box location of the qrcode
			(x, y, w, h) = qrcode.rect
			cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
	
			# the qrcode data is a bytes object so if we want to draw it
			# on our output image we need to convert it to a string first
			qrcodeData = qrcode.data.decode("utf-8")
			qrcodeType = qrcode.type

			# draw the qrcode data and qrcode type on the image
			text = "{} ({})".format(qrcodeData, qrcodeType)
			cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

			# Get the corners of the qrcode
			points = qrcode.polygon
			#print("The First Point: ", points[0])
			#print("The First Point: ", points[0].x, points[0].y)

			# Number of points in the convex hull
			n = len(points)
	
			# Draw the convext hull and marking the points
			for j in range(0,n):
				cv2.line(frame, points[j], points[ (j+1) % n], (255,0,0), 2)
				cv2.circle(frame, points[j], 5, (0, 255, 255), -1)
				point_number = "{}".format(j)
				cv2.putText(frame, point_number, points[j], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
			

			# Finding 3D coordinates of qrcode
			indexes = [i.start() for i in re.finditer(",", qrcodeData)]

			string_x = qrcodeData[indexes[0]+1 : indexes[1]]
			R_x = float(string_x)
			#print("R_x: ", R_x)

			string_y = qrcodeData[indexes[1]+1 : indexes[2]]
			R_y = float(string_y)
			#print("R_y: ", R_y)
	
			string_z = qrcodeData[indexes[2]+1 : len(qrcodeData)]
			R_z = float(string_z)
			#print("R_z: ", R_z)

			# Finding the coordinates x, y of the camera related to qrcode
			# the real diagonal distance of the qrcode
			R_D = 0.281
			# the diagonal distance of the qrcode in the image
			I_D = math.sqrt((points[2].x - points[0].x)*(points[2].x - points[0].x) + (points[2].y - points[0].y)*(points[2].y - points[0].y))
			
			# Finding the distance from the center of the QR-code to the position of image in the image
			# the center of the qrcode in the image
			qrcode_center_x = (points[0].x + points[1].x)/2
			qrcode_center_y = (points[0].y + points[1].y)/2

			image_distance = math.sqrt(math.pow((qrcode_center_x - image_center_x), 2) + math.pow((qrcode_center_y - image_center_y), 2))
			
			# the distance from the center of the QR-code to the robot in the real-world domain
			real_distance = image_distance * R_D/I_D
			#print("Real Distance: ", real_distance)
			
			image_height = qrcode_center_y - image_center_y
			image_base = qrcode_center_x - image_center_x
			# the coordinates of the robot in the real-world domain
			camera_pose_x = R_x - real_distance * image_base/image_distance
			camera_pose_y = R_y - real_distance * image_height/image_distance
		
			# Finding the theta angle
			delta_x = points[1].x - points[0].x
			delta_y = points[1].y - points[0].y
			print("delta_x, delta_y: ", delta_x, delta_y)

			if (delta_y == 0) and (delta_x >= 0):
				camera_pose_theta = math.pi/2
			elif (delta_y == 0) and (delta_x < 0):
				camera_pose_theta = - math.pi/2
			else:
				camera_pose_theta = math.atan2(delta_x, delta_y)
		
			#print("Camera Pose: x, y, theta: ", camera_pose_x, camera_pose_y, camera_pose_theta)

			usbcamera_pose = Pose()
   			point = Point()
      		quat = Quaternion()
        		
          		point.x = camera_pose_x
			point.y = camera_pose_y
			usbcamera_pose.position = Point(point.x, point.y, 0)
      		
        		quat = quaternion_from_euler(0.0, 0.0, camera_pose_theta)
            	usbcamera_pose.orientation = Quaternion(*quat)
                  
                  pub_camera_pose.publish(usbcamera_pose)
        		rospy.loginfo("USB Camera Pose:" + str(usbcamera_pose))

		# show the output frame
  		cv2.imshow("qrcode Scanner", frame)
		key = cv2.waitKey(1) & 0xFF
	
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break

	# close the output CSV file do a bit of cleanup
	print("[INFO] cleaning up...")
	csv.close()
	cv2.destroyAllWindows()
	vs.stop()

if __name__ == '__main__':
      try:
      	qrcode_localization()
    	except rospy.ROSInterruptException:
      	pass