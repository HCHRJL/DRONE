#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from TDK_function import Analysis, LineDetect

AL = Analysis()
LD = LineDetect()

hsv_red = ((0, 8, 29), (28, 129, 74)), ((121, 9, 32), (180, 117, 72))
hsv_green = (105, 166, 73), (121, 221, 149)
hsv_line = (61, 41, 16), (109, 211, 87)

HSV_MIN = (hsv_red[0][0], hsv_red[1][0]), hsv_green[0], hsv_line[0]
HSV_MAX = (hsv_red[0][1], hsv_red[1][1]), hsv_green[1], hsv_line[1]
KERNEL = np.ones((5,5), np.uint8)

STATE = "A Point"
DIRECTION = "backward and right"
target = 320, 240

def callback_state(msg):
	global STATE
	STATE = msg.data


def callback_img(img):
	global DIRECTION, target
	frame = CvBridge().compressed_imgmsg_to_cv2(img)
	frame = cv.flip(frame, -1)
	hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	binary_red = cv.inRange(hsv, HSV_MIN[0][0], HSV_MAX[0][0]) | cv.inRange(hsv, HSV_MIN[0][1], HSV_MAX[0][1])
	binary_green = cv.inRange(hsv, HSV_MIN[1], HSV_MAX[1])
	binary_line = cv.inRange(hsv, HSV_MIN[2], HSV_MAX[2])
	morph_red = cv.morphologyEx(binary_red, cv.MORPH_OPEN, KERNEL)
	morph_green = cv.morphologyEx(binary_green, cv.MORPH_OPEN, KERNEL)
	morph_line = cv.morphologyEx(binary_line, cv.MORPH_OPEN, KERNEL)
	all_morph = morph_red | morph_green | morph_line

	contour_red, _ = cv.findContours(morph_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	contour_green, _ = cv.findContours(morph_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	contour_line, _ = cv.findContours(morph_line, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	print(STATE)
	if STATE == "A Point" and len(contour_red)>0:
		see_A, index_A = AL.checkContour(contour_red, 1000)
		if see_A:
			center, radius = cv.minEnclosingCircle(contour_red[index_A])
			cv.circle(frame, (int(center[0]), int(center[1])), (int(radius)), (0,255,0), 3)
			target = center[0], center[1]
			PUB_STATE.publish(STATE)
	elif STATE == "B Point" and len(contour_green)>0:
		see_B, index_B = AL.checkContour(contour_green, 1000)
		if see_B:
			center, radius = cv.minEnclosingCircle(contour_green[index_B])
			cv.circle(frame, (int(center[0]), int(center[1])), (int(radius)), (0,255,0), 3)
			target = center[0], center[1]
			PUB_STATE.publish(STATE)

#################################################################################################################
### Hover #######################################################################################################
#################################################################################################################

	elif STATE == "hover"

		#if here get the sthdata,it will be stop 
		#try to use to change and control from the lider


	
		MSG_STATE = 
		PUB_STATE.publish(STATE)

#################################################################################################################
### Move ########################################################################################################
#################################################################################################################

	elif STATE == "move"


		#try to pub the sth to the next level that is a move level	
		#give the speed to PID and hold the 0speed down  
		#when it speed as 0 ,and wait for 5 secord or more
		###first
		#try to move to next side on the y side,give it a little speed like 5 speed for r and l lidar
		###secord
		#try to get the lidar'sth that can know the sth beside the drone  and stop it when it to move over that
		

		SPEED_target = 0,-2

		MSG_STATE = 
		PUB_STATE.publish(STATE)

#################################################################################################################
### FIND ########################################################################################################
#################################################################################################################

	elif len(contour_line)>0:
		see_line, index_line = AL.checkContour(contour_line, 1000)
		if see_line:
			mask = np.zeros((480, 640), np.uint8)
			rect = cv.boundingRect(contour_line[index_line])
			cv.drawContours(mask, [contour_line[index_line]], -1, 255, -1)
			roi = mask[rect[1]:(rect[1]+rect[3]), rect[0]:(rect[0]+rect[2])]
			clip, contour_1, contour_2 = LD.half_clip(size=rect, image=roi)

			rotate_1 = cv.minAreaRect(contour_1[0])
			rotate_2 = cv.minAreaRect(contour_2[0])
			box_1 = np.int0(cv.boxPoints(rotate_1))
			box_2 = np.int0(cv.boxPoints(rotate_2))
			line_or_round = LD.line_check(rotate_1[2], rotate_2[2])
			
			if clip == "left&right":
				for p in range(4):
					box_1[p][0] += rect[0]
					box_1[p][1] += rect[1]
					box_2[p][0] += rect[0]+int(roi.shape[1]/2)
					box_2[p][1] += rect[1]
				if line_or_round == "line":
					if rotate_1[1][0] > rotate_1[1][1]:
						if DIRECTION == "forward and left":
							target = int((box_1[0][0] + box_1[1][0]) / 2), int((box_1[0][1] + box_1[1][1]) / 2)
						elif DIRECTION == "backward and right":
							target = int((box_2[2][0] + box_2[3][0]) / 2), int((box_2[2][1] + box_2[3][1]) / 2)
					elif rotate_1[1][0] <= rotate_1[1][1]:
						if DIRECTION == "forward and left":
							target = int((box_1[0][0] + box_1[1][0]) / 2), int((box_1[0][1] + box_1[1][1]) / 2)
						elif DIRECTION == "backward and right":
							target = int((box_2[2][0] + box_2[3][0]) / 2), int((box_2[2][1] + box_2[3][1]) / 2)
					cv.line(frame, (320, 240), (target[0], target[1]), (0, 180, 0), 3)
				elif line_or_round == "round":
					if DIRECTION == "forward and left":
						target = int(rotate_1[0][0]), int(rotate_1[0][1])
					elif DIRECTION == "backward and right":
						target = int(rotate_2[0][0]), int(rotate_2[0][1])
					cv.line(frame, (320, 240), (target[0], target[1]), (0, 180, 0), 3)

			elif clip == "up&down":
				for p in range(4):
					box_1[p][0] += rect[0]
					box_1[p][1] += rect[1]
					box_2[p][0] += rect[0]
					box_2[p][1] += rect[1]+int(roi.shape[0]/2)
				if line_or_round == "line":
					if rotate_1[1][0] > rotate_1[1][1]:
						if DIRECTION == "forward and left":
							target = int((box_1[2][0] + box_1[3][0]) / 2), int((box_1[2][1] + box_1[3][1]) / 2)
						elif DIRECTION == "backward and right":
							target = int((box_2[0][0] + box_2[3][0]) / 2), int((box_2[0][1] + box_2[3][1]) / 2)
					elif rotate_1[1][0] <= rotate_1[1][1]:
						if DIRECTION == "forward and left":
							target = int((box_1[1][0] + box_1[2][0]) / 2), int((box_1[1][1] + box_1[2][1]) / 2)
						elif DIRECTION == "backward and right":
							target = int((box_2[0][0] + box_2[1][0]) / 2), int((box_2[0][1] + box_2[1][1]) / 2)
					cv.line(frame, (320, 240), (target[0], target[1]), (0, 180, 0), 3)
				elif line_or_round == "round":
					if DIRECTION == "forward and left":
						target = int(rotate_1[0][0]), int(rotate_1[0][1])
					elif DIRECTION == "backward and right":
						target = int(rotate_2[0][0]), int(rotate_2[0][1])
					cv.line(frame, (320, 240), (target[0], target[1]), (0, 180, 0), 3)
			cv.rectangle(frame, rect, (0,255,255), 3)
			cv.drawContours(frame, [box_1], -1, (255,0,0), 2)
			cv.drawContours(frame, [box_2], -1, (0,0,255), 2)
	MSG_CENTER.x = target[0]
	MSG_CENTER.y = target[1]
	PUB_CENTER.publish(MSG_CENTER)
	cv.imshow("frame", frame)
	cv.imshow("line", morph_line)
	cv.imshow("all_morph", all_morph)
	cv.waitKey(1)


if __name__ == "__main__":
	rospy.init_node("van_daag", anonymous=True)
	rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, callback_img, queue_size=1)
	rospy.Subscriber("/state_van_pid", String, callback_state, queue_size=10)
	PUB_CENTER = rospy.Publisher("/center", Point, queue_size=1)
	PUB_STATE = rospy.Publisher("/state_van_img", String, queue_size=1)
	MSG_CENTER = Point()
	MSG_STATE = String()
	rospy.spin()