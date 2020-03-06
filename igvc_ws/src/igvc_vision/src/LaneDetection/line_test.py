import cv2
import numpy as np
from camera_info import CameraInfo

def process_frame(img):

    RAW_WIDTH = 640
    RAW_HEIGHT = 480
    CROPPED_WIDTH = 640
    CROPPED_HEIGHT = 360

    camera_info = CameraInfo(53, 40, 76, 180, 217, CROPPED_WIDTH, CROPPED_HEIGHT)

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
    HSV_img = cv2.inRange(HSV_img, (0, 0, 80), (255, 80, 255))
    cv2.imshow('hsv 180', HSV_img)

    #gaussian_img = cv2.GaussianBlur(HSV_img, (9,9), 0)

    # kernel_size = 5
    # kernel = np.ones((kernel_size, kernel_size), np.uint8)
    # morph_img = cv2.morphologyEx(HSV_img, cv2.MORPH_OPEN, kernel)

    flat_img = camera_info.convertToFlat(HSV_img)

    return flat_img



                          
isCamera = True

if (isCamera):
    cam = cv2.VideoCapture('test3.avi')
    ret, img = cam.read()
    while ret:
        ret, img = cam.read()
        cv2.imshow('before', img)
        if not ret:
            break
        img2 = process_frame(img)
        cv2.imshow("final",img2)
        if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
            break  
    cam.release()
    cv2.waitKey(0)
    cv2.destroyAllWindows()

else:
    img = cv2.imread('test2.jpg')
    cv2.imshow('before', img)
    img2 = process_frame(img)
    cv2.imshow('final', img2)

cv2.waitKey(0)
cv2.destroyAllWindows()
