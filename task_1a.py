'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			BM_1424
# Author List:		Uzma Khan, Shairin Meraj, Abbas Haider, Faizan Choudhary
# Filename:			task_1a.py
# Functions:		detect_shapes, get_labeled_image (predefined),
# 					centroid_cnt, centroid_img, colors_detected, shapes (utility)


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################

def centroid_cnt(contour):
    """
    Purpose:
    ---
    This function takes contour as an argument and returns a list containing
    the x and y values of the centroid

    Input Arguments:
    ---
    `contour` : [ numpy array ]
            numpy array of (x, y) coordinates of boundary points of the object.

    Returns:
    ---
    `centroid` : [ list ]
            list of (x,y) coordinates of the centroid of the contours
    """
    
    centroid = []

    M = cv2.moments(contour)
    if M['m00'] != 0.0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        centroid.append((cx,cy))

    return centroid

def centroid_img(img):
    """
    Purpose:
    ---
    This function takes image as an argument and returns a list containing
    the x and y values of the centroid

    Input Arguments:
    ---
    `contour` : [ numpy array ]
            numpy array of (x, y) coordinates of boundary points of the object.

    Returns:
    ---
    `centroid` : [ list ]
            list of (x,y) coordinates of the centroid of the contours
    """
    centroid = []

    # converting image to grayscale
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Thresholding
    _, threshold = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

    # Contouring
    contours,_ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # to loop over the contours
    i=0
    for cnt in contours:
    
        # here we are ignoring first counter because 
        # findContours function detects whole image as shape
        if i==0:
            i=1
            continue

        M = cv2.moments(cnt)
        if M['m00'] != 0.0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

        centroid.append((cx,cy))

    return centroid


def colors_detected(img):
    """
    Purpose:
    ---
    This function takes the image as argument and returns a dictionary
    denoting the color of the shapes in the image.

    Input Arguments:
    ---
    `img` : [ numpy array ]
            numpy array of image returned by cv2 library

    Returns:
    ---
    `detected_colors` : {dictionary}
            dictionary containing details of colors present in image
    """
    detected_colors = {}

    centroids = centroid_img(img)

    for cent in centroids:
        
        # centroid    
        cx = cent[0]
        cy = cent[1]

        # finding out blue, green and red values for the pixel
    
        # image[y, x] is the correct syntax based on the fact that 
        # the x-value is the column number (i.e., width), and 
        # the y-value is the row number (i.e., height).
        (b, g, r) = img[cy][cx]
        
        if (b==0 and g==0 and r==255):
            detected_colors[cx,cy] = ['Red']
        elif (b==0 and g==255 and r==0):
            detected_colors[cx,cy] = ['Green']
        elif (b==255 and g==0 and r==0):
            detected_colors[cx,cy] = ['Blue']
        else:
            detected_colors[cx,cy] = ['Orange']

    return detected_colors


def shapes(img):
    
    img_shapes = {}
    # converting image to grayscale
    image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Thresholding
    _, threshold = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

    # Contouring
    contours,_ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #to loop over the contours
    i=0
    for cnt in contours:
    
        # ignoring first counter because 
        # findcontour function detects whole image as one shape
        if i==0:
            i=1
            continue

        # approximating
        approx = cv2.approxPolyDP(cnt, 0.025*cv2.arcLength(cnt, True), True)
        

        centroids = centroid_cnt(cnt)

        # centroid    
        cx = centroids[0][0]
        cy = centroids[0][1]
        
        # putting shape name
        if len(approx) == 3:
            img_shapes[cx,cy] = ['Triangle']
  
        elif len(approx) == 4:
        
            # for distinguishing between rectangle and square
            x, y, w, h = cv2.boundingRect(approx)

            aspectRatio = float (w)/h
            if aspectRatio >= 0.95 and aspectRatio <= 1.05:
                img_shapes[cx,cy] = ['Square']
            else:
                img_shapes[cx,cy] = ['Rectangle']

        elif (len(approx) == 5):
             img_shapes[cx,cy] = ['Pentagon']
  
        else:
             img_shapes[cx,cy] = ['Circle']

    return img_shapes


##############################################################

def detect_shapes(img):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list
	containing details of colored (non-white) shapes in that image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	Returns:
	---
	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image
	
	Example call:
	---
	shapes = detect_shapes(img)
	"""    
	detected_shapes = []

	##############	ADD YOUR CODE HERE	##############
	
	color = colors_detected(img)

	shape = shapes(img)
    
	keys_color = list(color.keys())
    
	keys_shape = list(shape.keys())
    
	details_list = []

	for key_shp in keys_shape:
		details_list = []
		cx_key = key_shp[0]
		cy_key = key_shp[1]

		for key_clr in keys_color:
			cx_k = key_clr[0]
			cy_k = key_clr[1]
            
			if (cx_k == cx_key and cy_k == cy_key):
				temp1 = (cx_k, cy_k)
				temp2 = (cx_key, cy_key)
				details_list.append(color.get(temp1)[0])
				details_list.append(shape.get(temp2)[0])
				details_list.append(temp2)
				detected_shapes.append(details_list)
				if len(details_list) == 3 :
					break
    
    
	##################################################
	
	return detected_shapes

def get_labeled_image(img, detected_shapes):
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the image and the detected shapes list as an argument
	and returns a labelled image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image

	Returns:
	---
	`img` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	img = get_labeled_image(img, detected_shapes)
	"""
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for detected in detected_shapes:
		colour = detected[0]
		shape = detected[1]
		coordinates = detected[2]
		cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
	return img

if __name__ == '__main__':
	
	# path directory of images in 'test_images' folder
	img_dir_path = 'test_images/'

	# path to 'test_image_1.png' image file
	file_num = 1
	img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
	
	# read image using opencv
	img = cv2.imread(img_file_path)
	
	print('\n============================================')
	print('\nFor test_image_' + str(file_num) + '.png')
	
	# detect shape properties from image
	detected_shapes = detect_shapes(img)
	print(detected_shapes)
	
	# display image with labeled shapes
	img = get_labeled_image(img, detected_shapes)
	cv2.imshow("labeled_image", img)
	cv2.waitKey(2000)
	cv2.destroyAllWindows()
	
	choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
	
	if choice == 'y':

		for file_num in range(1, 16):
			
			# path to test image file
			img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
			
			# read image using opencv
			img = cv2.imread(img_file_path)
	
			print('\n============================================')
			print('\nFor test_image_' + str(file_num) + '.png')
			
			# detect shape properties from image
			detected_shapes = detect_shapes(img)
			print(detected_shapes)
			
			# display image with labeled shapes
			img = get_labeled_image(img, detected_shapes)
			cv2.imshow("labeled_image", img)
			cv2.waitKey(2000)
			cv2.destroyAllWindows()
