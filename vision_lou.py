import cv2
import numpy as np
import math as m

# CONSTANT INIT

MAP_WIDTH = 1070 # mm
THYMIO_WIDTH = 110 # mm
global pix_per_mm # ratio between pixel and mm

# Lower and Higher range value of color for filter

RED_L= np.array([133,50,130]) 
RED_H = np.array([179,255,255])

GREEN_L = np.array([0, 0, 140],np.uint8)
GREEN_H = np.array([90, 90, 255],np.uint8)

BLUE_L = np.array([0, 90, 120],np.uint8)
BLUE_H = np.array([110, 255, 255],np.uint8)

YELLOW_L = np.array([0, 0, 158],np.uint8)
YELLOW_H = np.array([179, 240, 255],np.uint8)



def colorFilter(img, lower_range, upper_range):
	"""
	This function is used to filter the color of an image. The pixels not in the color range will turn black

        Args:
			img: image to filter
            lower range: [Hmin,Smin,Vmin] lower HSV value for the chosen color
			upper range: [Hmax,Smax,Vmax] upper HSV value for the chosen color
            
        Returns:
            filtered_img: filtered image
    """
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,lower_range,upper_range) # Create a mask with range
	filtered_img = cv2.bitwise_and(img,img,mask = mask)  # Performing bitwise and operation with mask in img variable

	return hsv, mask, filtered_img

def mapTransform(img,lower_range,upper_range):
	"""
	This function is used to find the edges of the map and correct the perspective

        Args:
			img: image taken from the camera
            lower range: [Hmin,Smin,Vmin] lower HSV value for the color of the frame 
			upper range: [Hmax,Smax,Vmax] upper HSV value for the color of the frame
            
        Returns:
            img_rescaled: image with corrected perspective and resized according to frame
			M: Perspective transform
			width: Image width in pixel
			height: Image height in pixel
    """
	hsv, mask, field = colorFilter(img,lower_range, upper_range)
	
	field_gray = cv2.cvtColor(field, cv2.COLOR_BGR2GRAY)
	# noise
	filtered_img = cv2.bilateralFilter(field_gray,3,75,75)
	# binary
	binary_img = cv2.threshold(filtered_img, 20, 255, cv2.THRESH_BINARY_INV)[1]

	# find contours of MAP
	contours, _ = cv2.findContours(binary_img.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	approx_cont = []
	
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt, 0.03*cv2.arcLength(cnt, True), True)
		area = cv2.contourArea(approx)
		# Criteria on the area to get only the external frame and not the obstacle or thymio if color filter did not work fine
		if (binary_img.shape[0]*binary_img.shape[1]/2 < area < binary_img.shape[0]*binary_img.shape[1]):
			approx_cont.append(approx.reshape(-1, 2))          
    
    # Correction of the perspective
	# First find the corners and figure out which one is which (top left / top right etc.)
	corners_tmp = approx_cont[0]
	corners_sorted = corners_tmp[corners_tmp[:, 1].argsort()]
	corners_top = corners_sorted[0:2,:]
	C_top_left = corners_top[corners_top[:, 0].argsort()][0,:]
	C_top_right = corners_top[corners_top[:, 0].argsort()][1,:]
	corners_bot = corners_sorted[2:,:]
	C_bot_left = corners_bot[corners_bot[:, 0].argsort()][0,:]
	C_bot_right = corners_bot[corners_bot[:, 0].argsort()][1,:]
	pts1 = np.float32([C_top_left, C_bot_left, C_bot_right, C_top_right])
    #Target Points
	t1 = pts1[0,0] #corner top left
	t2 = pts1[0,1]
	r1 = pts1[3,0] #corner top right
	r2 = pts1[3,1]
	b1 = pts1[2,0] #corner bottom right
	b2 = pts1[2,1]
	width = m.floor(m.sqrt((r1 - t1) * 2 + (r2 - t2) * 2))
	height = m.floor(m.sqrt((b1 - r1) * 2 + (b2 - r2) * 2))
	pts2 = np.float32([[0, 0], [0, height], [width, height], [width, 0]])
	M = cv2.getPerspectiveTransform(pts1.astype(np.float32), pts2)
	dst = cv2.warpPerspective(img, M, (width, height))

	## Rescaling image (between width and height)
	img_rescaled = dst[0:height, 0:width]
	return img_rescaled, M, width, height

def mm2pixRatio(img):
	"""
	This function is used to know how many mm is one pixel

        Args:
			img: image 
    """
	global pix_per_mm
	# Conversion mm to pixel
	pix_width = img.shape[1]
	pix_per_mm = pix_width / MAP_WIDTH