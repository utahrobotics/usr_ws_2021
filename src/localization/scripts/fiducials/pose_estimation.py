#!/usr/bin/env python
'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''

from rospkg import RosPack
import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
import rospy
import tf
import os
from geometry_msgs.msg import PoseStamped, Point, Quaternion

pose1Pub = rospy.Publisher('sensors/fiducial/pose1', PoseStamped, queue_size=10)
pose2Pub = rospy.Publisher('sensors/fiducial/pose2', PoseStamped, queue_size=10)
aruco1_offset = -0.25
aruco2_offset = 0.25
br = tf.TransformBroadcaster()
rp = RosPack()


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
	'''
	frame - Frame from the video stream
	matrix_coefficients - Intrinsic matrix of the calibrated camera
	distortion_coefficients - Distortion coefficients associated with your camera

	return:-
	frame - The frame with the axis drawn on it
	'''
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
	parameters = cv2.aruco.DetectorParameters_create()
	
	
	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
																cameraMatrix=matrix_coefficients,
																distCoeff=distortion_coefficients)
	
	# If markers are detected
	if len(corners) > 0:
		for i in range(0, len(ids)):
			id = ids[i]
			# Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
			#TODO: edit fiducial length to reflect reality
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.053, matrix_coefficients,
																		   distortion_coefficients)
			print()
		print(rvec)
		print(tvec)
		# Draw a square around the markers
		cv2.aruco.drawDetectedMarkers(frame, corners)
		
		# Draw Axis
		cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
		
		pose = PoseStamped()
		
		pose.header.stamp = rospy.get_rostime()
		#TODO: update the frame_id to the camera
		pose.header.frame_id = "map"
		
		pose.pose.position.x = tvec[0][0][2]
		pose.pose.position.y = -tvec[0][0][0] + aruco1_offset if id == 1 else aruco2_offset
		pose.pose.position.z = -tvec[0][0][1]
		
		quaternion = tf.transformations.quaternion_from_euler(rvec[0][0][0], rvec[0][0][1], rvec[0][0][2])
		inverse_quaternion = tf.transformations.quaternion_from_euler(0, 0, -rvec[0][0][2])
		#type(pose) = geometry_msgs.msg.Pose
		pose.pose.orientation.x = inverse_quaternion[0]
		pose.pose.orientation.y = inverse_quaternion[1]
		pose.pose.orientation.z = inverse_quaternion[2]
		pose.pose.orientation.w = inverse_quaternion[3]
		
		print(id)
		
		if id == 1:
			pose1Pub.publish(pose)
			br.sendTransform((tvec[0][0][2], -tvec[0][0][0] + aruco1_offset, -tvec[0][0][1]), inverse_quaternion, rospy.Time.now(), "aruco1_pose_est", "aruco1")
		elif id == 2:
			pose2Pub.publish(pose)
			br.sendTransform((tvec[0][0][2], -tvec[0][0][0] + aruco2_offset, -tvec[0][0][1]), inverse_quaternion, rospy.Time.now(), "aruco3_pose_est", "aruco3")

	return frame


if __name__ == '__main__':
	rospy.init_node("fiducials")
	type = "DICT_5X5_100"
	
	aruco_dict_type = ARUCO_DICT[type]
	path = rp.get_path('localization')
	calibration_matrix_path = os.path.join(path, "scripts/fiducials/calibration_matrix.npy")
	distortion_coefficients_path = os.path.join(path, "scripts/fiducials/distortion_coefficients.npy")
	
	k = np.load(calibration_matrix_path)
	d = np.load(distortion_coefficients_path)
	
	video = cv2.VideoCapture(0)
	time.sleep(2.0)
	
	while True:
		ret, frame = video.read()
		
		if not ret:
			break
		
		output = pose_estimation(frame, aruco_dict_type, k, d)
		
		cv2.imshow('Estimated Pose', output)
		
		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
	
	video.release()
	cv2.destroyAllWindows()
