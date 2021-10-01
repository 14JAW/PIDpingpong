# Author: Joel Akers
# Type: Python Programming Course
# Purpose: This program allows the student to study the Python
# programming language and collect data to understand PID controllers more thoroughly.
# It uses image to find a ping pong ball and returns it's location between two points.

# Usage: The user will install openCV for python and numpy. The user must
# have a webcam or camera on the computer. When run, a window with the live
# video feed will appear. The user will Click and Drag from the top of the
# tube to the bottom of the tube. The program will display the fraction of
# distance between the top and bottom of where the ORANGE ping pong ball
# is located

import cv2
import numpy as np
import math
import xlwt
from xlwt import Workbook
import datetime
import os

# Global Variables
# expected top and bottom pixels. Will work upside down
top_pixel = (5, 100)
bottom_pixel = (100, 100)

# holds the expected path of the ball through the image
setNewPath = False
path = []
past_locations = []

# Get neightbors for filtering or for blowing up features
# returns the 8 neighbors of the selected pixel


def Neighbors(image, pixel):
    neighbors = []
    x, y = pixel[:]
    neighbors.append((x - 1, y - 1))
    neighbors.append((x, y - 1))
    neighbors.append((x + 1, y - 1))
    neighbors.append((x - 1, y))
    neighbors.append((x + 1, y))
    neighbors.append((x - 1, y + 1))
    neighbors.append((x, y + 1))
    neighbors.append((x + 1, y + 1))
    return neighbors

# Checks to make sure the pixel is in the range of the image


def PixelValid(image, pixel):
    if pixel[0] < 0 or pixel[1] < 0 or pixel[0] >= image.shape[0] or pixel[1] >= image.shape[1]:
        return False
    return True

# returns a list with the path inside it.


def GivePath(image, start, stop):

    # Handles the case where the user clicked one pixel.
    if start == stop:
        return

    # get global path variable
    global path
    path = []

    # if line rule fails the path, then see from other direction
    if start[0] == stop[0]:
        # equation for determining where line is on image
        slope = (start[0] - stop[0])/(start[1] - stop[1])
        intercept = start[0] - slope * start[1]

        # start point
        x = start[0]
        y = start[1]

        # Add pixels to path as long as in image and is not the stop pixel
        # It will pass through stop pixel because of equations
        while PixelValid(image, (x, y)) and not (x == stop[0] and y == stop[1]):
            path.append((x, y))
            if start[1] > stop[1]:
                y -= 1
            else:
                y += 1
            x = int(slope * y + intercept)
    else:
        # Same theory here, just flipped
        slope = (start[1] - stop[1])/(start[0] - stop[0])
        intercept = start[1] - slope * start[0]

        x = start[0]
        y = start[1]
        while PixelValid(image, (x, y)) and not (x == stop[0] and y == stop[1]):
            path.append((x, y))
            if start[0] > stop[0]:
                x -= 1
            else:
                x += 1
            y = int(slope * x + intercept)


def SelectPath(event, y, x, flags, param):
    # Click and drag mouse from start to stop in order to select path
    if event == cv2.EVENT_LBUTTONDOWN:
        global top_pixel
        top_pixel = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        global bottom_pixel
        bottom_pixel = (x, y)

        # dont have access to image here so we set a global boolean
        # to find the path
        global setNewPath
        setNewPath = True

# returns pythagorean distance between two points


def PythagoreanDist(x, y):
    if len(x) == 1 and len(y) == 1:
        return math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    elif len(x) == 2 and len(y) == 2:
        return math.sqrt(math.pow(x[0] - y[0], 2) + math.pow(x[1] - y[1], 2))
    elif len(x) == 3 and len(y) == 3:
        return math.sqrt(math.pow(x[0] - y[0], 2) + math.pow(x[1] - y[1]) + math.pow(x[2] - y[2]))
    else:
        print("Pythagorean: data type mismatch or more than three dimenstions")
        return 0


def PingPongHeight():
    # sample period (seconds)
    period = 0.5

    # excel Doc
    wb = Workbook()
    sheet = wb.add_sheet('Sheet')
    row = 0
    column = 0
    sheet.write(row, column, 'sample #')
    sheet.write(row, column + 1, 'distance')
    sheet.write(row, column + 2, 'time difference')

    row = row + 1
    # camera object
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    ret, image = cap.read()
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)

    # Set user ability to redraw path
    cv2.setMouseCallback('image', SelectPath)
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)

    startTime = datetime.datetime.now()
    previousSampleTime = startTime
    sampleNumber = 1
    while(True):
        # grab image
        ret, image = cap.read()
        # resize to make calculations faster
        image = cv2.resize(image, (240, 120))
        # convert to hsv and flip image accross y axis to make it a mirror
        image_hsv = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), 1)
        image = cv2.flip(image, 1)

        # ensure these are the global variables
        global setNewPath
        global top_pixel
        global bottom_pixel
        global path
        global past_locations

        # if is true then refind the path
        if setNewPath is True:
            setNewPath = False
            GivePath(image, top_pixel, bottom_pixel)

        # set conditions for what pingpong ball looks like
        # This uses HSV: Hue, Saturation, value
        orangeMin = np.array([10, 100, 20], np.uint8)
        orangeMax = np.array([25, 255, 255], np.uint8)
        threshold = cv2.inRange(image_hsv, orangeMin, orangeMax)

        # show threshold for debugging
        location_start = (-1, -1)
        location_end = (-1, -1)

        # go through each of the pixels in path
        for pixel in range(1, len(path) - 2):

            # if three pixels in the path in a row are orange, then display as blue
            # This will begin allowing us to determine where the pingpong ball actually is.
            if threshold[path[pixel][0], path[pixel][1]] != 0  \
                and threshold[path[pixel - 1][0], path[pixel - 1][1]] != 0  \
                    and threshold[path[pixel + 1][0], path[pixel + 1][1]] != 0:

                if location_start == (-1, -1):
                    location_start = path[pixel]
                location_end = path[pixel]
                image[path[pixel][0], path[pixel][1]] = (255, 0, 0)

                # color neighbors blue to make easier to see
                for items in Neighbors(image, path[pixel]):
                    if PixelValid(image, items):
                        image[items[0], items[1]] = (255, 0, 0)
        distance = 0
        # Gives value start of the blue to the end of the blue
        # and finds center by averaging
        if location_start[0] != -1 and location_start[1] != -1 \
                and location_end[0] != -1 and location_end[1] != -1:
            location = (int((location_start[0] + location_end[0])/2),
                        int((location_start[1] + location_end[1])/2))
            past_locations.append(location)
            # color center position red for visibility
            for items in Neighbors(image, location):
                if PixelValid(image, items):
                    image[items[0], items[1]] = (0, 0, 255)

            # if ping pong ball is out of view, return as 0
            # set k to the distance that the end points are separated by
            # in real life
            k = 1
            if location != location_end:
                distance = k * PythagoreanDist(
                    location, bottom_pixel)/PythagoreanDist(top_pixel, bottom_pixel)

        # display top and bottom pixels
        for items in Neighbors(image, top_pixel):
            if PixelValid(image, items):
                image[items[0], items[1]] = (0, 255, 0)

        for items in Neighbors(image, bottom_pixel):
            if PixelValid(image, items):
                image[items[0], items[1]] = (0, 255, 0)

        # for visual effect, the position of the ball is displayed as a percent
        # of the path it is from the bottom to the top of the path.
        font = cv2.FONT_HERSHEY_SIMPLEX
        org_perc = (20, 20)
        # fontScale
        fontScale = 0.5
        color = (255, 0, 0)
        thickness = 1
        image = cv2.putText(image, str(distance) + '%', org_perc, font,
                            fontScale, color, thickness, cv2.LINE_AA)
        # Display the resulting frame
        cv2.resizeWindow('image', 600, 600)
        cv2.imshow('image', image)

        currentTime = datetime.datetime.now()
        if (currentTime - previousSampleTime).total_seconds() >= period:
            sheet.write(sampleNumber, column, sampleNumber)
            sheet.write(sampleNumber, column + 1, distance)
            sheet.write(sampleNumber, column + 2,
                        (currentTime - previousSampleTime).total_seconds())
            previousSampleTime = currentTime
            sampleNumber = sampleNumber + 1
        # allows user to quit the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # release camera
    cap.release()

    # finds available workbook number to avoid deleting previous data
    workbookName = 'OrangeData'
    workbookNum = 0
    while True:
        files = os.listdir('./')
        tempName = workbookName + str(workbookNum) + '.xls'
        if tempName in files:
            workbookNum = workbookNum + 1
        else:
            wb.save(tempName)
            break


# Program actually starts here
PingPongHeight()

cv2.destroyAllWindows()
