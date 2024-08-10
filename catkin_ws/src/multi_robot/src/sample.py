import cv2
import numpy as np
import matplotlib.pyplot as plt

cv_image = cv2.imread('multi_robot/src/raw_img.png')

cv2.imshow("Image window", cv_image)
cv2.waitKey(0)

# gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
blue_img, green_img, red_img = cv2.split(cv_image)
 
# blur = cv2.GaussianBlur(red_img, (5, 5), cv2.BORDER_DEFAULT)
ret, thresh = cv2.threshold(red_img, 58, 255, cv2.THRESH_BINARY)


cv2.imshow("thresh.png",thresh)
cv2.waitKey(0)

# hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

# lower_red = np.array([0,10,20])
# upper_red = np.array([15,255,255])

# lower = [1, 0, 20]
# upper = [60, 40, 200]

# lower = np.array(lower, dtype="uint8")
# upper = np.array(upper, dtype="uint8")

# mask = cv2.inRange(cv_image, lower, upper)

contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# if len(contours) > 0:
#   red_areas = max(contours, key=cv2.contourArea)
#   x, y, w, h = cv2.boundingRect(red_areas)
#   cv2.rectangle(mask,(x, y),(x+w, y+h),(0, 0, 255), 2)

# rect = cv2.minAreaRect(contours)
# box = cv2.boxPoints(rect)
# box = np.int0(box)
# cv2.drawContours(mask,[box],0,(0,0,255),2)

# blank = np.zeros(thresh.shape[:2], dtype='uint8')
# cnt = contours[4]
# cv2.drawContours(thresh, contours, -1, (0, 0, 255), 1)

# cnt = contours[0]
# print(len(contours))
for c in contours:
    rect = cv2.minAreaRect(c)
    
    area=rect[1][0]*rect[1][1]
    if area>150 and area<400:
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        cv_image = cv2.drawContours(cv_image,[box],0,(0,255,0),2)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print(cx,cy)
            cv2.circle(cv_image, (cx, cy), 1, (0, 0, 255), -1)


# cv2.imshow("Mask", mask)
# cv2.waitKey(0)
cv2.imshow("Show", cv_image)
cv2.waitKey(0)