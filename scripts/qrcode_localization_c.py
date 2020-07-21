# USAGE
# python qrcode_scanner.py

# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import time
import cv2
import re
#from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()
vs = VideoStream(src=2).start()
time.sleep(2)

qrcode_dim = 0.2

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 600 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=600)

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

		#2D image points
		image_points = np.array([
                            			(points[0].x, points[0].y),
                            			(points[1].x, points[1].y),
                            			(points[2].x, points[2].y),
                            			(points[3].x, points[3].y)
                        		], dtype="double")
  		
    		# Find 3D coordinates of qrcode
		indexes = [i.start() for i in re.finditer(",", qrcodeData)]

		string_x = qrcodeData[indexes[0]+1 : indexes[1]]
		ORG_x = float(string_x)
		#print("ORG_x: ", ORG_x)

		string_y = qrcodeData[indexes[1]+1 : indexes[2]]
		ORG_y = float(string_y)
		#print("ORG_y: ", ORG_y)
  
		string_z = qrcodeData[indexes[2]+1 : len(qrcodeData)]
		ORG_z = float(string_z)
		#print("ORG_z: ", ORG_z)

		# Find 3D coordinates of 4 corners
		point_0_x = ORG_x - qrcode_dim/2
		point_0_y = ORG_y - qrcode_dim/2
		point_0_z = ORG_z
  
		point_1_x = ORG_x - qrcode_dim/2
		point_1_y = ORG_y + qrcode_dim/2
		point_1_z = ORG_z
  
		point_2_x = ORG_x + qrcode_dim/2
		point_2_y = ORG_y + qrcode_dim/2
		point_2_z = ORG_z
  
		point_3_x = ORG_x + qrcode_dim/2
		point_3_y = ORG_y - qrcode_dim/2
		point_3_z = ORG_z
  
		model_points = np.array([
                            			(point_0_x, point_0_y, point_0_z),
                            			(point_1_x, point_1_y, point_1_z),
                            			(point_2_x, point_2_y, point_2_z),
                            			(point_3_x, point_3_y, point_3_z)
                        		])
  
		# Camera internals
		# Camera Matrix 
		camera_matrix = np.array(
                         			[[829.58836, 0, 316.86144],
                         			[0, 831.40965, 230.45752],
                         			[0, 0, 1]], 
                            			dtype = "double"
                            		)
		# distortion coefficients
		dist_coeffs = np.array(
                         			[0.053771, 0.258723, 0.001084, -0.003260, 0.000000], 
                            			dtype = "double"
                            		)

		#Use solvePnP to get the rotation and translation vector of the QR code relative to the camera
		#(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, None, None, False, cv2.SOLVEPNP_ITERATIVE)
		(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
		#print "Rotation Vector:\n {0}".format(rotation_vector)
		#print "Translation Vector:\n {0}".format(translation_vector)
		
  		#Get 3x3 rotation matrix
		rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
		#print(rotation_matrix)

		#Zero out view matrix
		view_matrix = np.zeros(shape=(4,4))
		
  		#Get opencv transfer matrix (camera -> object)
		for row in range(3):
			for col in range(3):
				view_matrix[row][col] = rotation_matrix[row][col]
			view_matrix[row][3] = translation_vector[row][0]
		view_matrix[3][3] = 1.0
		#print("View Matrix: ", view_matrix)

		#Invert matrix to get position and orientation of camera relative to qrcode
		camera_matrix = np.linalg.inv(view_matrix)
		#print("Camera Matrix: ", camera_matrix)

		# Finding orientation matrix of the camera relative to qrcode
		cam_rotation_matrix = np.zeros(shape=(3,3))
		for row in range(3):
      			for col in range(3):
				cam_rotation_matrix[row][col] = camera_matrix[row][col]
		#print("camera rotation matrix: ", cam_rotation_matrix)
  
		# Finding translation vector of the camera relative to qrcode
		cam_translation_vector = np.zeros(3)
		for row in range(3):
			cam_translation_vector[row] = camera_matrix[row][3]
		#print("camera translation vector: ", cam_translation_vector)
		
		# Finding translation vector of the camera relative to qrcode
    		cam_rotation_vector, _ = cv2.Rodrigues(cam_rotation_matrix)
		#print("camera rotation vector: ", cam_rotation_vector)

		# Finding camera pose in the world frame
		cam_pose_x = cam_translation_vector[0]
		cam_pose_y = cam_translation_vector[1]
		cam_pose_theta = cam_rotation_vector[2]
		cam_pose = np.array([cam_pose_x, cam_pose_y, cam_pose_theta])
		print("Camera Pose: [x, y, theta]: ", cam_pose)
		


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