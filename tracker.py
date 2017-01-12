from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2
import numpy as np

print "OpenCV OK"
print ("Version " + cv2.__version__)
main_width = 256
disp_width = 160

# nominal
#blueLower = (100, 150, 0)
#blueUpper = (140, 255, 255)
# adjusted

# Blue LED threshold values (HSV)
blueLower = (80, 200, 40)
blueUpper = (120, 255, 255)

# Red LED threshold values (HSV)
# Two ranges are needed for full coverage
redLowerA = (160, 100, 40)
redUpperA = (180, 255, 255)
redLowerB = (0, 100, 40)
redUpperB = (30, 255, 255)

use_background_subtraction = False
font_default = cv2.FONT_HERSHEY_PLAIN

print "Start video stream."
# Initialize video stream
vs = PiVideoStream().start()
cam = vs.camera

print "Configuring camera."
# Optimize camera for bright LEDs
cam.iso = 200
cam.shutter_speed = 1500
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
    frame = imutils.resize(frame, width = main_width)

    disp = frame.copy()

    text = "WAIT FOR CALIBRATION"
    scale = 1
    thickness = 2
    textsize= cv2.getTextSize(text, font, scale, thickness)[0]
    height, width = frame.shape[:2]
    textorg = ((width - textsize[0])/2, (height - textsize[1])/2)
    
    cv2.putText(disp, text, textorg, font_default, scale, (255, 255, 255), thickness)

    disp = imutils.resize(disp, width = disp_width)
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


# heuristic function that determines the exact positions
# of the quadcopter's props
hist_A1 = []
hist_A2 = []
hist_B1 = []
hist_B2 = []
def process_points(bpts, rpts):

    centroid = (0, 0)
    A1 = (0, 0)
    A2 = (0, 0)
    B1 = (0, 0)
    B2 = (0, 0)

    # find an initial centroid
    allpts = np.vstack((bpts, rpts))
    if allpts.size == 0:
        return centroid, A1, A2, B1, B2
    length = allpts.shape[0]
    sum_x = np.sum(allpts[:, 0])
    sum_y = np.sum(allpts[:, 1])
    raw_centroid = (sum_x/length, sum_y/length)
    centroid = (int(sum_x/length), int(sum_y/length))

    # TEMP: Reject if not all points perfectly visible...
    if allpts.size != 8:
        return centroid, A1, A2, B1, B2

    # assign points
    # Sort the points by relative position

    furthest_right = 0
    furthest_down = 0
    furthest_left = 0
    furthest_up = 0
    for i, p in enumerate(allpts):
        if p[0] > allpts[furthest_right][0]:
            furthest_right = i
        if p[1] > allpts[furthest_down][1]:
            furthest_down = i
        if p[0] < allpts[furthest_left][0]:
            furthest_left = i
        if p[1] < allpts[furthest_up][1]:
            furthest_up = i
    blue_right = allpts[furthest_right] in bpts
    blue_down = allpts[furthest_down] in bpts

    # Identify the points

    if blue_right and blue_down:
        B2 = tuple(allpts[furthest_right])
        A2 = tuple(allpts[furthest_down])

        B1 = tuple(allpts[furthest_left])
        A1 = tuple(allpts[furthest_up])

    if blue_right and not blue_down:
        A2 = tuple(allpts[furthest_right])
        B1 = tuple(allpts[furthest_down])

        A1 = tuple(allpts[furthest_left])
        B2 = tuple(allpts[furthest_up])

    if not blue_right and blue_down:
        A1 = tuple(allpts[furthest_right])
        B2 = tuple(allpts[furthest_down])

        A2 = tuple(allpts[furthest_left]) 
        B1 = tuple(allpts[furthest_up])

    if not blue_right and not blue_down:
        B1 = tuple(allpts[furthest_right])
        A1 = tuple(allpts[furthest_down])

        B2 = tuple(allpts[furthest_left])
        A2 = tuple(allpts[furthest_up])

    # Package and return proper points
    A1 = (int(A1[0]), int(A1[1])) 
    A2 = (int(A2[0]), int(A2[1])) 
    B1 = (int(B1[0]), int(B1[1])) 
    B2 = (int(B2[0]), int(B2[1])) 
    return centroid, A1, A2, B1, B2


# main tracking loop
while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=main_width)

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #delta = cv2.absdiff(first_frame, gray)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    gmask = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
    gmask = cv2.threshold(gmask, 50, 255, cv2.THRESH_BINARY)[1]

    # Create blue mask
    bmask = cv2.inRange(hsv, blueLower, blueUpper)
    #bmask = cv2.erode(bmask, None, iterations=2)
    bmask = cv2.dilate(bmask, None, iterations=2)

    # Create two red masks with two ranges, then combine them
    rmaskA = cv2.inRange(hsv, redLowerA, redUpperA)
    rmaskB = cv2.inRange(hsv, redLowerB, redUpperB)
    rmask = cv2.bitwise_or(rmaskA, rmaskB)
    #rmask = cv2.erode(rmask, None, iterations=2)
    rmask = cv2.dilate(rmask, None, iterations=2)

    rect_size = 8

    # Compute contours
    bcnts = cv2.findContours(bmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    rcnts = cv2.findContours(rmask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    cnt_center = None

    bpts = np.empty(shape=[0, 2])
    rpts = np.empty(shape=[0, 2])

    if len(bcnts) > 0:
        i = 2;
        for c in bcnts:
            if i == 0:
                break
            #i = i - 1

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            cnt_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            bpts = np.append(bpts, [cnt_center], axis=0)

            if radius < 100:
                p1 = (int(x) - rect_size, int(y) - rect_size)
                p2 = (int(x) + rect_size, int(y) + rect_size)
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 1)
                #cv2.drawContours(frame, [c], 0, (200, 0, 0), 2)
                #cv2.circle(frame, cnt_center, 5, (0, 255, 0), -1)

    if len(rcnts) > 0:
        i = 2;
        for c in rcnts:
            if i == 0:
                break
            #i = i - 1

            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            cnt_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            rpts = np.append(rpts, [cnt_center], axis=0)

            if radius < 100:
                p1 = (int(x) - rect_size, int(y) - rect_size)
                p2 = (int(x) + rect_size, int(y) + rect_size)
                cv2.rectangle(frame, p1, p2, (0, 0, 255), 1)
                #cv2.drawContours(frame, [c], 0, (0, 0, 200), 2)
                #cv2.circle(frame, cnt_center, 5, (0, 255, 0), -1)
   
    # process point results
    # blue points are in bpts[], red points in rpts[]
    center, posA1, posA2, posB1, posB2 = process_points(bpts, rpts) 

    # draw representation of quad
    cv2.circle(frame, center, 3, (255, 255, 255), 1)
    cv2.circle(frame, posA1, 2, (255, 255, 255), 1)
    cv2.circle(frame, posA2, 2, (255, 255, 255), 1)
    cv2.circle(frame, posB1, 2, (255, 255, 255), 1)
    cv2.circle(frame, posB2, 2, (255, 255, 255), 1)
    cv2.putText(frame, "A1", posA1, font_default, 1, (255, 255, 255), 1)
    cv2.putText(frame, "A2", posA2, font_default, 1, (255, 255, 255), 1)
    cv2.putText(frame, "B1", posB1, font_default, 1, (255, 255, 255), 1)
    cv2.putText(frame, "B2", posB2, font_default, 1, (255, 255, 255), 1)

    # compute angle
    fwd = ((posA2[0]+posB2[0])/2, (posA2[1]+posB2[1])/2)
    if center[0] == fwd[0]:
        heading = 180
    else:
        heading = np.arctan(float((center[1]) - fwd[1]) / float(center[0] - fwd[0])) * 57.2957795131


    if posA1 == posA2 == posB1 == posB2 == (0, 0):
        print "NO TRACK"
    else:
        print "TRACKING"
        cv2.line(frame, center, posA1, (255, 255, 255), 1)
        cv2.line(frame, center, posA2, (255, 255, 255), 1)
        cv2.line(frame, center, posB1, (255, 255, 255), 1)
        cv2.line(frame, center, posB2, (255, 255, 255), 1)
        cv2.arrowedLine(frame, center, fwd, (150, 150, 150), 1)
 
    # print console output
    print "Position: " + str(center)
    print "Heading: " + str(heading)[:6]
    print "------------"

    
    # show FPS
    cv2.putText(frame, str(FPS), (0, 12), font_default, 1, (0, 80, 80), 1)

    # display the frame
    frame = imutils.resize(frame, width = disp_width)
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
