# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import sys

#orange buoy default hue lower and upper limits
# 0-3 and 170-180


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
ap.add_argument("-ub", "--upper",
	help="upper limit for HSV value")
ap.add_argument("-lb", "--lower",
	help="lower limit for HSV value")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	cap = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
else:
	cap = cv2.VideoCapture(args["video"])


#if an upper HSV value is provided
if not args.get("upper",False):
	upper = (180,200,255)

#otherwise, use default upper value to detect orange hue (taken from orange buoy)
else:
	upper = np.array(map(int, args["upper"].split(',')))

#if a lower HSV value is provided
if not args.get("lower", False):
	lower = (170,20,20)

#otherwise, use default lower value to detect orange hue
else:
	lower = np.array(map(int, args["lower"].split(',')))




while(1):
		
    # Take each frame
	_, frame = cap.read()

    # Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower, upper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if(len(cnts)>0):

		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 20:
			print(len(cnts))
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	

    # Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)

	cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()
