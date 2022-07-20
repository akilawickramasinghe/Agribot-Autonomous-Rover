from pickle import FALSE, TRUE
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import math

import warnings
'''
with warnings.catch_warnings():
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    import md5, sha

''' 

def warp(img):

    #Define calibration box in source and destination coordinates
    img_size = (img.shape[1],img.shape[0])
    #Four Source coordinates
    src = np.float32(
        [[193, 151],
         [53, 470],
         [550, 470],
         [423, 151], ])
    # Four desired coordinations
    dst = np.float32(
        [[40, 151],
         [49, 470],
         [550, 470],
         [495, 151], ])
    #Compute the perspective transform
    M = cv.getPerspectiveTransform(src,dst)
    #Compute the inverse also by swapping the input parameters
    Minv = cv.getPerspectiveTransform(dst, src)
    #Create warped image - using linear interpolation
    warped = cv.warpPerspective(img, M, img_size, flags=cv.INTER_LINEAR)

    return warped

def Extractgreen(src):
    fsrc = np.array(src, dtype=np.float32) / 255.0
    (b, g, r) = cv.split(fsrc)
    gray = 2 * g - b - r

    # Find the maximum and minimum
    (minVal, maxVal, minLoc, maxLoc) = cv.minMaxLoc(gray)

    # Calculate histogram
    hist = cv.calcHist([gray], [0], None, [256], [minVal, maxVal])
    #plt.plot(hist)
    #plt.show()

    # Convert to u8 type, perform otsu binarization
    gray_u8 = np.array((gray - minVal) / (maxVal - minVal) * 255, dtype=np.uint8)
    (thresh, bin_img) = cv.threshold(gray_u8, -1.0, 255, cv.THRESH_OTSU)
    cv.imshow('bin_img', bin_img)

    # Get colored image
    (b8, g8, r8) = cv.split(src)
    color_img = cv.merge([b8 & bin_img, g8 & bin_img, r8 & bin_img])
    return bin_img

def do_segment1(frame):
    # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
    # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
    height = frame.shape[0]
    # Creates a triangular polygon for the mask defined by three (x, y) coordinates
    polygons = np.array([

                            [(150, 473), (320, 196), (390, 196), (540, 473)]

                        ])
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(frame, mask)
    return segment

def do_canny1(frame):
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    (thresh, im_bw) = cv.threshold(gray, 237, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    blur = cv.GaussianBlur(im_bw, (5, 5), 0)
    canny = cv.Canny(blur, 5, 255, apertureSize=3)
    size = np.size(canny)  # returns the product of the array dimensions
    skel = np.zeros(canny.shape, np.uint8)  # array of zeros
    ret, canny1 = cv.threshold(canny, 237, 255, 0)  # thresholding the image
    element = cv.getStructuringElement(cv.MORPH_CROSS, (3, 3))
    done = False
    eroded = cv.erode(canny1, element)
    temp = cv.dilate(eroded, element)
    temp = cv.subtract(canny1, temp)
    skel = cv.bitwise_or(skel, temp)
    return skel

def calculate_lines1(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    mode="Guide"
    try:
        for line in lines:
            # Reshapes line from 2D array to 1D array
            x1, y1, x2, y2 = line.reshape(4)
            # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_intercept = parameters[1]
            # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
            if slope < 0:
                left.append((slope, y_intercept))
            else:
                right.append((slope, y_intercept))
            mode="Guide"
    except:
        print("No center crop line detected")
        mode="Auto"


    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    left_avg = np.average(left, axis = 0)
    right_avg = np.average(right, axis = 0)
    # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
    left_line = calculate_coordinates1(frame, left_avg)
    right_line = calculate_coordinates1(frame, right_avg)
    centerline= (left_line+right_line)/2
    #print(type(right_avg))
    return np.array([centerline]),mode

def calculate_coordinates1(frame, parameters):
    try:
        slope, intercept = parameters
    except TypeError:
        slope, intercept = 0.1, 0
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 - 150)
    # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
    x1 = int((y1 - intercept) / slope)
    # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
    x2 = int((y2 - intercept) / slope)
    #print(np.array([x1, y1, x2, y2]))
    return np.array([x1, y1, x2, y2])

def visualize_lines1(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_visualize = np.zeros_like(frame)
    mode="Guided"
    try:
        # Checks if any lines are detected
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                # Draws lines between two coordinates with green color and 5 thickness
                cv.line(lines_visualize, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 15)
    except:
        mode="Auto"

    return lines_visualize,x1, y1, x2, y2,mode

def houghtranform1(canny,frame):
    hough = cv.HoughLinesP(canny, 2, np.pi / 180, 90, np.array([]), minLineLength=100, maxLineGap=50)

    # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
    lines,mode= calculate_lines1(frame, hough)



    # Visualizes the lines
    lines_visualize, x1, y1, x2, y2 ,mode= visualize_lines1(frame, lines)


    # print(x1, y1, x2, y2)

    # Overlays lines on frame by taking their weighted sums and adding an arbitrary scalar value of 1 as the gamma argument
    output = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)
    output = warp(output)
    cv.line(output, pt1=(320, 0), pt2=(320, 480), color=(0, 0, 255), thickness=10)

    xc, yc = (320, 480)  # center point bottom of center line
    dist = math.sqrt((xc - x1) ** 2 + (yc - y1) ** 2)
    dist2 = (xc - x1)
    rad = math.atan(dist2 / 480)
    deg = math.degrees(rad)
    deg = round(deg, 2)
    cv.putText(img=output, text=str(deg) + "deg", org=(150, 250), fontFace=cv.FONT_HERSHEY_TRIPLEX, fontScale=1,
               color=(200, 0, 0), thickness=1)

    return output,deg,mode


############### function to detect crop rows ###############################################################################

def do_canny2(frame):
    frame = cv.normalize(frame, None, alpha=0, beta=200, norm_type=cv.NORM_MINMAX)
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    (thresh, im_bw) = cv.threshold(gray, 200, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    blur = cv.GaussianBlur(im_bw, (5, 5), 0)

    canny = cv.Canny(blur, 5, 255, apertureSize=5)
    size = np.size(canny)  # returns the product of the array dimensions
    skel = np.zeros(canny.shape, np.uint8)  # array of zeros
    ret, canny1 = cv.threshold(canny, 170, 255, 0)  # thresholding the image
    element = cv.getStructuringElement(cv.MORPH_CROSS, (3, 3))
    done = False
    eroded = cv.erode(canny1, element)
    temp = cv.dilate(eroded, element)
    temp = cv.subtract(canny1, temp)
    skel = cv.bitwise_or(skel, temp)

    return skel

def do_segment2(frame):

    # Since an image is a multi-directional array containing the relative intensities of each pixel in the image, we can use frame.shape to return a tuple: [number of rows, number of columns, number of channels] of the dimensions of the frame
    # frame.shape[0] give us the number of rows of pixels the frame has. Since height begins from 0 at the top, the y-coordinate of the bottom of the frame is its height
    height = frame.shape[0]
    # Creates a triangular polygon for the mask defined by three (x, y) coordinates
    polygons = np.array([
        [(100, 330), (230, 140), (350, 140), (500, 330)]

    ])
    polygon2 = np.array([
        [(150, 480), (262, 0), (390, 0), (514, 480)]

    ])

    polygon3 = np.array([
        [(0,110), (0, 0), (640, 0), (640, 110)]

    ])

    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(frame)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask2 = np.zeros_like(frame)
    mask2.fill(255)
    mask3 = np.zeros_like(frame)
    mask3.fill(255)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask2, polygon2, 0)
    cv.fillPoly(mask3,polygon3, 0)

    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(frame, mask)
    segment2 = cv.bitwise_and(frame, mask2)
    segment3 = cv.bitwise_and(mask3, segment2)
    return segment3

def calculate_lines2(frame, lines):
    # Empty arrays to store the coordinates of the left and right lines
    left = []
    right = []
    # Loops through every detected line
    #if lines.any() == None:
     #   print("No crop row Detected : Robot is not in track")

    for line in lines:

        # Reshapes line from 2D array to 1D array
        x1, y1, x2, y2 = line.reshape(4)
        # Fits a linear polynomial to the x and y coordinates and returns a vector of coefficients which describe the slope and y-intercept
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
        # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
        if slope < 0:
            left.append((slope, y_intercept))
        else:
            right.append((slope, y_intercept))

    # Averages out all the values for left and right into a single slope and y-intercept value for each line
    left_avg = np.average(left, axis = 0)
    right_avg = np.average(right, axis = 0)
    # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
    left_line = calculate_coordinates2(frame, left_avg)
    right_line = calculate_coordinates2(frame, right_avg)
    return np.array([left_line, right_line])

def calculate_coordinates2(frame, parameters):
    try:
        slope, intercept = parameters
    except TypeError:
        slope, intercept = 0.1,0
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 - 150)
    # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
    x1 = int((y1 - intercept) / slope)
    # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def visualize_lines2(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    lines_visualize = np.zeros_like(frame)

    try:
     # Checks if any lines are detected
     if lines is not None:
        for x1, y1, x2, y2 in lines:

            # Draws lines between two coordinates with green color and 5 thickness
            cv.line(lines_visualize, (int(x1), int(y1+10)), (int(x2), int(y2+10)), (0, 255, 0), 15)
     else:
         print("No crop line detected")
    except:

        print("No crop line detected")

    return lines_visualize

def houghTransformation2(segment,frame):
    global hough_out
    hough = cv.HoughLinesP(segment, 2, np.pi / 180, 100, np.array([]), minLineLength=170, maxLineGap=70)
    # print(type(hough))
    # Averages multiple detected lines from hough into one line for left border of lane and one line for right border of lane
    try:
        lines = calculate_lines2(frame, hough)

        # Visualizes the lines
        lines_visualize = visualize_lines2(frame, lines)
        # Overlays lines on frame by taking their weighted sums and adding an arbitrary scalar value of 1 as the gamma argument
        hough_out = cv.addWeighted(frame, 0.9, lines_visualize, 1, 1)

        #cv.imshow("Crop Row Detection", output)
        # out.write(output)
        field_state = TRUE

    except:
        print("No crop line detected")
        field_state = FALSE

    return field_state

def detect(frame):
    canny2 = do_canny2(frame)
    segment2 = do_segment2(canny2)
    row_state = houghTransformation2(segment2, frame)
    #cv.imshow("Output",hough_out)
    #print("FIELD STATE", hough_output2)
    if row_state:
        return TRUE
    else:
        return FALSE

def getAngle(frame):
    canny = do_canny1(frame)
    segment = do_segment1(canny)
    hough_output_angle, deg,mode = houghtranform1(segment, frame)
    cv.imshow("Angle",hough_output_angle)
    return deg

