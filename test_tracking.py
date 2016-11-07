from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2

# nominal
#blueLower = (100, 150, 0)
#blueUpper = (140, 255, 255)
# adjusted

# Blue LED threshold values (HSV)
blueLower = (80, 60, 60)
blueUpper = (120, 255, 255)

# Red LED threshold values (HSV)
# Two ranges are needed for full coverage
redLowerA = (160, 10, 80)
redUpperA = (180, 255, 255)
redLowerB = (0, 10, 80)
redUpperB = (30, 255, 255)

use_background_subtraction = False
font_default = cv2.FONT_HERSHEY_PLAIN

# Initialize video stream
vs = PiVideoStream().start()
cam = vs.camera

# Optimize camera for bright LEDs
cam.iso = 100
cam.shutter_speed = 2000
time.sleep(2.0)

# Disable AWB algorithm for true color
g = cam.awb_gains
cam.awb_mode = 'off'
cam.awb_gains = g

# initialize background subtraction frame
first_frame = None

# wait for background subtraction
start_time = time.time();
while True:
    if not use_background_subtraction:
        break
    frame = vs.read()
    frame = imutils.resize(frame, width = 400)

    disp = frame.copy()

    text = "WAIT FOR CALIBRATION"
    scale = 1
    thickness = 2
    textsize= cv2.getTextSize(text, font, scale, thickness)[0]
    height, width = frame.shape[:2]
    textorg = ((width - textsize[0])/2, (height - textsize[1])/2)
    
    cv2.putText(disp, text, textorg, font_default, scale, (255, 255, 255), thickness)

    cv2.imshow("Calibrating...", disp)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break


    # after 2 seconds end
    if time.time() - start_time >= 2:
        first_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.destroyAllWindows()
        break;


# initialize FPS counter
start_time = time.time()
frames = 0
FPS = 0

# main tracking loop
while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #delta = cv2.absdiff(first_frame, gray)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    gmask = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    gmask = cv2.threshold(gmask, 50, 255, cv2.THRESH_BINARY)[1]

    # Create blue mask
    bmask = cv2.inRange(hsv, blueLower, blueUpper)
    bmask = cv2.erode(bmask, None, iterations=2)
    bmask = cv2.dilate(bmask, None, iterations=2)

    # Create two red masks with two ranges, then combine them
    rmaskA = cv2.inRange(hsv, redLowerA, redUpperA)
    rmaskB = cv2.inRange(hsv, redLowerB, redUpperB)
    rmask = cv2.bitwise_or(rmaskA, rmaskB)
    rmask = cv2.erode(rmask, None, iterations=2)
    rmask = cv2.dilate(rmask, None, iterations=2)

    rect_size = 20

    # Compute contours
    bcnts = cv2.findContours(bmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    rcnts = cv2.findContours(rmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None

    if len(bcnts) > 0:
        i = 2;
        for c in bcnts:
            if i == 0:
                break
            #i = i - 1

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius < 100:
                p1 = (int(x) - rect_size, int(y) - rect_size)
                p2 = (int(x) + rect_size, int(y) + rect_size)
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 1)
                #cv2.drawContours(frame, [c], 0, (200, 0, 0), 2)
                #cv2.circle(frame, center, 5, (0, 255, 0), -1)

    if len(rcnts) > 0:
        i = 2;
        for c in rcnts:
            if i == 0:
                break
            #i = i - 1

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius < 100:
                p1 = (int(x) - rect_size, int(y) - rect_size)
                p2 = (int(x) + rect_size, int(y) + rect_size)
                cv2.rectangle(frame, p1, p2, (0, 0, 255), 1)
                #cv2.drawContours(frame, [c], 0, (0, 0, 200), 2)
                #cv2.circle(frame, center, 5, (0, 255, 0), -1)

    # show FPS
    cv2.putText(frame, "FPS: " + str(FPS), (0, 12), font_default, 1, (255, 255, 255), 1)

    # display the frame
    cv2.imshow("Tracker", frame)

    # update FPS counter
    if time.time() - start_time >= 1:
        start_time = time.time()
        FPS = frames
        frames = 0
    else:
        frames = frames + 1


    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop()
