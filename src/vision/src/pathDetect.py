import cv2
import numpy as np
import math


img = cv2.imread('path3.png', 1)

h, w, c = img.shape

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
blur = cv2.GaussianBlur(hsv,(5,5),0)

lower_red = np.array([0,0,0])
upper_red = np.array([70,220,200])

mask = cv2.inRange(blur, lower_red, upper_red)
# res = cv2.bitwise_and(imgb, imgb, mask= mask)
# res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
# res = cv2.threshold(res, 0, 150, cv2.THRESH_BINARY)

kernel = np.ones((4,4),np.uint8)
res = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
im2, contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cnt = contours[0]

M = cv2.moments(cnt)

cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])

cv2.circle(img,(cx,cy), 4, (0,0,255), -1)
cv2.circle(img,(int(w/2),int(h/2)), 4, (0,0,255), -1)
# cv2.line(img,(cx,cy),(int(w/2), int(h/2)),(255,0,0),2)

cv2.circle(img,(w,h), 4, (0,0,255), -1)
# cv2.circle(img,(w,0), 4, (0,0,255), -1)
# cv2.circle(img,(0,h), 4, (0,0,255), -1)
cv2.circle(img,(0,0), 4, (0,0,255), -1)

# cv2.drawContours(img, contours, -1, (0,255,0), 3)   
rect = cv2.minAreaRect(cnt)
(x,y),(width,height),theta = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)

print(theta)

cv2.drawContours(img,[box],0,(0,0,255),2)
pt1 = (box[0][0], box[0][1])
pt4 = (box[3][0], box[3][1])
pt2 = (box[1][0], box[1][1])
pt3 = (box[2][0], box[2][1])

print(pt1)
print(pt2)
print(pt3)
print(pt4)

x1,y1 = (pt4[0] + pt1[0])/2.0, (pt4[1] + pt1[1])/2.0
x2,y2 = (pt2[0] + pt3[0])/2.0, (pt2[1] + pt3[1])/2.0

cv2.line(img,(int(x1),int(y1)),(int(x2),int(y2)),(255,0,0),2)
cv2.line(img,(int(w/2),0),(int(w/2),h),(255,0,0),2)

pt1 = (box[0][0], h-box[0][1])
pt4 = (box[3][0], h-box[3][1])
pt2 = (box[1][0], h-box[1][1])
pt3 = (box[2][0], h-box[2][1])

x1,y1 = (pt4[0] + pt1[0])/2.0, (pt4[1] + pt1[1])/2.0
x2,y2 = (pt2[0] + pt3[0])/2.0, (pt2[1] + pt3[1])/2.0

# print(x1 - x2)
angle = float(y2-y1)/float(x2-x1)
angle = math.atan2(y2-y1,x2-x1)
print (math.degrees(angle))    

cv2.imshow('res', img)

cv2.waitKey(0)
cv2.destroyAllWindows()


# ret2,th2 = cv2.threshold(img_gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)