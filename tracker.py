from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2
import numpy as np
import serial
import math

print "OpenCV OK"
print ("Version " + cv2.__version__)
main_width = 160
disp_width = 160

# nominal
#blueLower = (100, 150, 0)
#blueUpper = (140, 255, 255)
# adjusted

# Blue LED threshold values (HSV)
blueLower = (80, 240, 40)
blueUpper = (120, 255, 255)

# Red LED threshold values (HSV)
# Two ranges are needed for full coverage
redLowerA = (160, 150, 40)
redUpperA = (180, 255, 255)
redLowerB = (0, 150, 40)
redUpperB = (30, 255, 255)

use_background_subtraction = False
font_default = cv2.FONT_HERSHEY_PLAIN

# get screen overlays
overlay1 = cv2.imread("/home/pi/quadtracker/img/overlay1.png")

# get a serial port
def getPort(portname, baud):
    try:
        return serial.Serial(portname, baud, timeout=0)
    except:
        print("SERIAL PORT NOT FOUND.")
        return None

serial_port = getPort("/dev/ttyUSB0", 115200)
if(serial_port != None):
    serial_port.isOpen()

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
debug_text = ""


# angle formed by A, C, with corner at point B
def get_ang(A, B, C):
    v0 = np.array(A) - np.array(B)
    v1 = np.array(C) - np.array(B)
    ang = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
    return np.degrees(ang)


# function to find center of points
def get_centroid(pts):
    length = pts.shape[0]
    sum_x = np.sum(pts[:, 0])
    sum_y = np.sum(pts[:, 1])
    raw_centroid = (sum_x/length, sum_y/length)
    return (sum_x/length, sum_y/length)


# alternative function to label points
def process_points2(bpts, rpts):
    centroid = (0.0, 0.0)
    A1 = (0.0, 0.0)
    A2 = (0.0, 0.0)
    B1 = (0.0, 0.0)
    B2 = (0.0, 0.0)

    # reject if data isn't complete
    if bpts.size != 4 or rpts.size != 4:
        return centroid, A1, A2, B1, B2

    # cast to float
    bpts = np.array(bpts) + 0.
    rpts = np.array(rpts) + 0.

    allpts = np.vstack((bpts, rpts))

    centroid = get_centroid(allpts)

    # assign points
    # get centroids of each color group
    blue_centroid = get_centroid(bpts)
    red_centroid = get_centroid(rpts)

    # make decision based on angles
    if(get_ang(centroid, blue_centroid, bpts[0]) >= 0):
        A2 = bpts[0]
        B2 = bpts[1]
    else:
        B2 = bpts[0]
        A2 = bpts[1]
    
    if(get_ang(centroid, red_centroid, rpts[0]) >= 0):
        A1 = rpts[0]
        B1 = rpts[1]
    else:
        B1 = rpts[0]
        A1 = rpts[1]
 
    # Package and return proper points
    #A1 = (int(A1[0]), int(A1[1])) 
    #A2 = (int(A2[0]), int(A2[1])) 
    #B1 = (int(B1[0]), int(B1[1])) 
    #B2 = (int(B2[0]), int(B2[1])) 
    return centroid, A1, A2, B1, B2


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

    allpts = np.vstack((bpts, rpts))

    # reject if data isn't complete
    if allpts.size != 8 or bpts.size != 4 or rpts.size != 4:
        return centroid, A1, A2, B1, B2

    centroid = get_centroid(allpts)

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
        if p[0] <= allpts[furthest_left][0]:
            furthest_left = i
        if p[1] <= allpts[furthest_up][1]:
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


# Euclidean distance
def dist(A, B):
    return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)


# drawing parameters
rect_size = 4
font_scale = 0.525

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
    center_f, posA1_f, posA2_f, posB1_f, posB2_f = process_points2(bpts, rpts) 
    center = (int(center_f[0]), int(center_f[1]))
    posA1 = (int(posA1_f[0]), int(posA1_f[1]))
    posA2 = (int(posA2_f[0]), int(posA2_f[1]))
    posB1 = (int(posB1_f[0]), int(posB1_f[1]))
    posB2 = (int(posB2_f[0]), int(posB2_f[1]))

    height, width = frame.shape[:2]

    # add overlay
    overlay_alpha = 0.75
    frame = cv2.add(overlay1, frame)
    #frame = cv2.addWeighted(frame, overlay_alpha, overlay, 1 - overlay_alpha, 0, frame)

    # draw representation of quad
    col_quadpoint = (255, 255, 255)
    col_quadtext = (200, 200, 200)
    col_quadline = (200, 200, 200)
    col_heading = (150, 150, 150)
    cv2.circle(frame, center, 2, col_quadpoint, 1)
    cv2.circle(frame, posA1, 1, col_quadpoint, 1)
    cv2.circle(frame, posA2, 1, col_quadpoint, 1)
    cv2.circle(frame, posB1, 1, col_quadpoint, 1)
    cv2.circle(frame, posB2, 1, col_quadpoint, 1)
    cv2.putText(frame, "A1", posA1, font_default, font_scale, col_quadtext, 1)
    cv2.putText(frame, "A2", posA2, font_default, font_scale, col_quadtext, 1)
    cv2.putText(frame, "B1", posB1, font_default, font_scale, col_quadtext, 1)
    cv2.putText(frame, "B2", posB2, font_default, font_scale, col_quadtext, 1)

    # compute guidance position
    pos_x = center_f[0] - width/2.0
    pos_y = center_f[1] - height/2.0

    # compute altitude
    altitude = 80 - (dist(center_f, posA1_f) + dist(center_f, posA2_f) + dist(center_f, posB1_f) + dist(center_f, posB2_f))/4.0

    # compute angle
    fwd = ((posA2_f[0]+posB2_f[0])/2, (posA2_f[1]+posB2_f[1])/2)
    heading = get_ang((center_f[0], center_f[1] - 10), center_f, fwd)
 
    # add bottom bar
    bottom_bar = np.zeros((10, frame.shape[1], 3), np.uint8)
    frame = np.concatenate((frame, bottom_bar), axis = 0)

    tracking = 0

    debug_text = pos_x

    # print if tracking
    if posA1 == posA2 == posB1 == posB2 == (0, 0):
        cv2.putText(frame, "NO TRACK", (55, 8), font_default, font_scale * 1.25, (255, 255, 255), 1)
    else:
        tracking = 1
        cv2.putText(frame, "TRACKING", (55, 8), font_default, font_scale * 1.25, (255, 255, 255), 1)
        cv2.line(frame, center, posA1, col_quadline, 1)
        cv2.line(frame, center, posA2, col_quadline, 1)
        cv2.line(frame, center, posB1, col_quadline, 1)
        cv2.line(frame, center, posB2, col_quadline, 1)
        cv2.arrowedLine(frame, center, (int(fwd[0]), int(fwd[1])), col_quadline, 1)
    
    cv2.putText(frame, (str(debug_text)), (40, 40), font_default, font_scale, (255, 255, 255), 1)

    # data format is <S-M--.--X--.--Y--.--H--.--A--CE>.
    data_string = 'S{}M{}X{}Y{}H{}A'.format(tracking, pos_x, pos_y, heading, altitude)
    data_string += '{}CE'.format(sum([ord(c) for c in data_string]) % (256))

    # send data over serial port
    # TODO: Hot plug serial port
    if(serial_port != None):
        #data_string = ",7777," + str(tracking) + "," + str(float(pos_x)) + "," + str(float(pos_y)) + "," + str(float(heading)) + "," + str(float(altitude)) + ",9999,"
        serial_port.write(data_string)

        # mirror serial in to console
        bytesIn = serial_port.inWaiting();
        if bytesIn: print(serial_port.read(bytesIn));

    #print data_string

    # print more info
    cv2.putText(frame, ("POS " + str(center)), (4, 114), font_default, font_scale, (255, 255, 255), 1)
    cv2.putText(frame, ("HDG " + str(heading)[:6]), (4, 124), font_default, font_scale, (255, 255, 255), 1)
    cv2.putText(frame, ("ALT " + str(altitude)[:6]), (60, 114), font_default, font_scale, (255, 255, 255), 1)
    # show FPS
    cv2.putText(frame, str(FPS), (4, 12), font_default, font_scale, (0, 120, 120), 1)
    # display the frame
    frame = imutils.resize(frame, width = disp_width)
    #cv2.imshow("Tracker", frame)

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

serial_port.close()
cv2.destroyAllWindows()
vs.stop()
