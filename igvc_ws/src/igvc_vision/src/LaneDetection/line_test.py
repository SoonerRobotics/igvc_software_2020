import cv2
import numpy as np
from camera_info import CameraInfo
import time
import math

min_width = 0
max_width = 1000000
min_perimeter = 1
min_area = 500
max_area = 30000
min_height = 0
max_height = 1000000
solidity = [21,100]
max_vertex_count = 1000000
min_vertex_count = 0
min_ratio = 0
max_ratio = 10000
output = []

def is_contour_valid(n):

   # x, y, w, h = cv2.boundingRect(n)

    # # Ignore if width or height is out of bounds
    # if (w < min_width or w > max_width):
    #     print('width')
    #     return False
    # if (h < min_height or h > max_height):
    #     print('height')
    #     return False
    

    # # Ignore if the perimeter of area is small enough
    # if (cv2.arcLength(n, True) < min_perimeter):
    #     print('arc')
    #     return False

    # Ignore if number of edges of contour is not what we expect
    # if (len(n) < min_vertex_count or len(n) > max_vertex_count):
    #     print('vertex')
    #     return False

    # ratio = (float)(w) / h
    # if (ratio < min_ratio or ratio > max_ratio):
    #     print('ratio')
    #     return False

    area = cv2.contourArea(n)
    # Ignore if area is small enough
    if (area > max_area):
        print('area')
        return False

    # hull = cv2.convexHull(n)
    # solid = 100 * area / cv2.contourArea(hull)
    # if (solid < solidity[0] or solid > solidity[1]):
    #     print('solid')
    #     return False
    
    return True

def hough_line_transform(img):
    dst = cv2.Canny(img, 50, 200, None, 3)
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    lines = cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)

    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

    #mask = np.ones(cdstP.shape[:2], dtype='uint8') * 0

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            cv2.line(mask, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)



    cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    #cv2.imshow("Detected Lines (in red)1 - Probabilistic Line Transform", mask)

def process_frame(img):

    RAW_WIDTH = 640
    RAW_HEIGHT = 480
    CROPPED_WIDTH = 640
    CROPPED_HEIGHT = 360

    camera_info = CameraInfo(53, 40, 76, 200, 217, CROPPED_WIDTH, CROPPED_HEIGHT)
    #camera_info = CameraInfo(0, 0,0,80,75, CROPPED_WIDTH, CROPPED_HEIGHT)

    dim = (RAW_WIDTH, RAW_HEIGHT)

    img = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    crop_ratio = float(CROPPED_HEIGHT) / float(RAW_HEIGHT)

    # Crops off the top 25% of the image`
    crop_img = img[int(RAW_HEIGHT * float(1 - crop_ratio)) : RAW_HEIGHT, 0 : RAW_WIDTH]

    median_img = cv2.medianBlur(crop_img, 7)

    HSV_img = cv2.cvtColor(median_img, cv2.COLOR_BGR2HSV)

    h, s, v = cv2.split(HSV_img)
    v = cv2.equalizeHist(v)
    HSV_img = cv2.merge((h,s,v))

    # Threshold image
    HSV_img = cv2.inRange(HSV_img, (0, 0, 20), (255, 90, 255))

    kernel_size = 5
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    morph_img = cv2.morphologyEx(HSV_img, cv2.MORPH_OPEN, kernel)
    morph_img = cv2.morphologyEx(morph_img, cv2.MORPH_GRADIENT, kernel)
    #morph_img = cv2.morphologyEx(HSV_img, cv2.MORPH_CLOSE, kernel)
    

    #morph_img = cv2.dilate(HSV_img, kernel, iterations=1)

    contours, hierarchy = cv2.findContours(HSV_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Array of all whites
    mask = np.ones(morph_img.shape[:2], dtype='uint8') * 255

    output = []
    for n in contours:
        if (not is_contour_valid(n)):
            cv2.drawContours(mask, [n], -1, 0, -1)
        else:
            output.append(n)
            

    masked = cv2.bitwise_and(morph_img, morph_img, mask=mask)
    cv2.imshow('Masking', mask)

    
    # #print contours
    # img = cv2.drawContours(masked, output, -1, (0,255,0), 3)

    flat_img = camera_info.convertToFlat(masked)

    #hough_line_transform(flat_img)

    return flat_img



                          
isCamera = True

if (isCamera):
    cam = cv2.VideoCapture('test3.avi')
    ret, img = cam.read()
    while ret:
        #time.sleep(0.05)
        ret, img = cam.read()
        cv2.imshow('before', img)
        if not ret:
            break
        img2 = process_frame(img)
        cv2.imshow("final",img2)
        if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
            break  
    cam.release()

else:
    img = cv2.imread('test2.jpg')
    cv2.imshow('before', img)
    img2 = process_frame(img)
    cv2.imshow('final', img2)

cv2.waitKey(0)
cv2.destroyAllWindows()
