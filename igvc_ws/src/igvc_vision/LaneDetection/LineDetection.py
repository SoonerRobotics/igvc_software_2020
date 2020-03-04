import cv2
import numpy as np
from camera_info import CameraInfo

def process_frame():
    #img = cv2.imread("test1.jpg")
    #cam = cv2.VideoCapture(0)
    cam = cv2.VideoCapture('test.avi')
    rawWidth = 640
    rawHeight = 480
    croppedWidth = 640
    croppedHeight = 360

    ret, img = cam.read()

    dim = (rawWidth,rawHeight)
    img=cv2.resize(img,dim,interpolation=cv2.INTER_AREA)
    cropRatio = float(croppedHeight)/float(rawHeight)

    camera_info = CameraInfo(53,40,76,180,217,croppedWidth,croppedHeight)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('output.avi',fourcc, 30.0, (640,360))
    while(ret == True):
        ret, img = cam.read()
        #img = cv2.equalizeHist(img)
        if not ret:
            break
        crop_img = img[int(rawHeight * float(1-cropRatio)):rawHeight, 0:rawWidth]  # crops off the top 25% of the image
        cv2.imshow("cropped", crop_img)#median blur
        img_medianBlur = cv2.medianBlur(crop_img,7)
        cv2.namedWindow('img_medianBlur',0)
        cv2.resizeWindow('img_medianBlur', 640, 480)
        #cv2.imshow('img_medianBlur',img_medianBlur)
        img_medianBlur = cv2.cvtColor(img_medianBlur, cv2.COLOR_BGR2HSV)
        #cv2.imshow('img_medianBlur',img_medianBlur)
        h,s,v = cv2.split(img_medianBlur)
        v = cv2.equalizeHist(v)
        img_HSV = cv2.merge((h,s,v))
        img_HSV = cv2.inRange(img_medianBlur, (20, 20, 80), (100, 80, 220))
        #filter to remove ramp lip
        kernel_size = 5
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        img_HSV = cv2.morphologyEx(img_HSV, cv2.MORPH_OPEN, kernel)
        cv2.imshow('img_HSV',img_HSV)
        img_gaussianBlur = cv2.GaussianBlur(img_HSV, (9,9),0)

        #top down view
        img_topDown = camera_info.convertToFlat(img_gaussianBlur)
        cv2.imshow("top Down", img_topDown)
        
        contours, hierarchy = cv2.findContours(img_topDown, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_width = 0
        max_width = 1000000
        min_perimeter = 1
        min_area = 1000
        min_height = 0
        max_height = 1000000
        solidity = [21,100]
        max_vertex_count = 1000000
        min_vertex_count = 0
        min_ratio = 0
        max_ratio = 10000
        output = []
        
        for n in contours:
            x, y, w, h = cv2.boundingRect(n)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(n)
            if (area < min_area):
                continue
            if (cv2.arcLength(n, True) < min_perimeter):
                continue
            hull = cv2.convexHull(n)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(n) < min_vertex_count or len(n) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(n)
        
        #print contours
        img = cv2.drawContours(img, output, -1, (0,255,0), 3)
        #out.write(img)
        if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
            break                        
                                # press q to quit

    cam.release()
    #out.release()
process_frame()

cv2.waitKey(0)
cv2.destroyALLWINDOWS()