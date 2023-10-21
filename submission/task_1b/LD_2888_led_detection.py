# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

# load the image, 
image = cv2.imread('led.jpg', 1)
cv2.imshow('LED', image)

# convert it to grayscale, and blur it
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# blur = cv2.GaussianBlur(gray, (111,111), cv2.BORDER_DEFAULT)
blur = cv2.blur(gray, (9,9))
cv2.imshow('blur', blur)

# threshold the image to reveal light regions in the blurred image
ret, thresh = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)
cv2.imshow('Threshold', thresh)

# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
eroded = cv2.erode(thresh, kernel, iterations = 3)
cv2.imshow('Eroded', eroded)

dilated = cv2.dilate(eroded, kernel, iterations = 3)
cv2.imshow('Dilated', dilated)

# cv2.waitKey(0)
# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components

labels = measure.label(dilated, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

# loop over the unique components
for label in np.unique(labels):
    # print(label)
	# if this is the background label, ignore it
    if label == 0:
        continue
	# otherwise, construct the label mask and count the number of pixels 
    else:
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = 255
        # numPixels = cv2.add(mask, labelMask)
        numPixels = labelMask.sum() // 255
	# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
 
    print(label, numPixels)
    if numPixels > 300:
        mask = cv2.add(mask, labelMask)
	
# find the contours in the mask, then sort them from left to right
contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)
contours = sorted(contours, key=cv2.contourArea, reverse=True)

# loop over the contours

# Initialize lists to store centroid coordinates and area
centroid_list = []
area_list = []

print(contours)

# Loop over the contours
for i, contour in enumerate(contours):

    # Calculate the area of the contour
    area = cv2.contourArea(contour)    

    # Draw the bright spot on the image
    M = cv2.moments(contour)
    centroid_x = int(M["m10"]/M["m00"])
    centroid_y = int(M["m01"]/M["m00"])
    centroid = (centroid_x, centroid_y)
    cv2.drawContours(image, [contour], -1, (0, 0, 255), 2)
    #cv2.circle(image, centroid, 5, (255, 0, 0), -1)
    

    # Append centroid coordinates and area to the respective lists
    centroid_list.append(centroid)
    area_list.append(area)

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", image)

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {len(contours)}\n")
    # Loop over the contours
    for i, centroid in enumerate(centroid_list):
        area = area_list[i] 
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")
# Close the text file
file.close()


