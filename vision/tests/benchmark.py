import cv2
import time
import platform
import numpy as np
import sys

def hasScreen():
  return platform.system() == "Darwin"# or platform.system() == "Linux"

from pkg_resources import parse_version
OPCV3 = parse_version(cv2.__version__) >= parse_version('3')

def millis():
  return int(round(time.time() * 1000))

old_time = millis()

kernel = np.ones((2,2),np.uint8)
img = cv2.imread("sample.png")

while(True):
  # Convert to HSV color space
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

  # define range of blue color in hs
  lower_hsv = np.array([66,55,67])
  upper_hsv = np.array([96,255,255])

  # do threshold
  mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

  # morhpology (erode + dilate)
  morphed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

  # contours (finding edges)
  _, contours, hierarchy = cv2.findContours(morphed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  # For python 2:  # contours, hierarchy = cv2.findContours(morphed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


  for contour in contours:
    hull = cv2.convexHull(contour)
    moment = cv2.moments(hull)
    centerOfMass = (int(round(moment['m10']/moment['m00'])) , int(round(moment['m01']/moment['m00'])))
    if hasScreen():
      cv2.drawContours(img, hull, -1, (0,0,255), 3)
      cv2.circle(img, centerOfMass, 5, (0,255,0))

  if hasScreen():
    cv2.imshow("input", img)

# release the capture
cap.release()
cv2.destroyAllWindows()
