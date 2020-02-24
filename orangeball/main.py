#!/usr/bin/env python3

import numpy as np
import cv2

image = cv2.imread('image_many.png')
original = image.copy()
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower = np.array([5, 50, 50], dtype = "uint8")
upper = np.array([15, 255, 255], dtype = "uint8")
mask = cv2.inRange(image, lower, upper)

# Find contours
cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Extract contours depending on OpenCV version
cnts = cnts[0] if len(cnts) == 2 else cnts[1]

# Sort by Area
cntsSorted = sorted(cnts, key = lambda x: cv2.contourArea(x), reverse = True)

############# Edited

cv2.drawContours(original, [cntsSorted[0]], -1, (36, 255, 12), -1)
#
# for c in cntsSorted:
#     perimeter = cv2.arcLength(c, True)
#     approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
#
#     cv2.drawContours(original, [c], -1, (36, 255, 12), -1)

############# From Stackoverflow

# # Iterate through contours and filter by the number of vertices
# for c in cntsSorted:
#     perimeter = cv2.arcLength(c, True)
#     approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
#
#     # cv2.drawContours(original, [c], -1, (36, 255, 12), -1)
#
#     if len(approx) > 5:
#         cv2.drawContours(original, [c], -1, (36, 255, 12), -1)

# cv2.imshow('mask', mask)
# cv2.imshow('original', original)

cv2.imwrite('mask.png', mask)
cv2.imwrite('original.png', original)
cv2.waitKey()