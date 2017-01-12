from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize camera and grab reference
camera = PiCamera()
rawCapture = PiRGBArray(camera)

#camera warmup
time.sleep(0.1)

#grab image
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

#display image
cv2.imshow("Image", image)
cv2.waitKey(0)
