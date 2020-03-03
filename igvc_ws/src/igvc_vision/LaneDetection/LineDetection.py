import cv2
import numpy as np
def perspective_transform(img):
    #does transform
    return img
def med_blur(img):
    img = cv2.medianBlur(img,5)
    return img
def hsv_thresh(img):
    


    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (90, 100, 50), (150, 255, 255))
    #frame_threshold = 1 - frame_threshold
    return frame_threshold


#img = cv2.imread("test1.jpg")
cap = cv2.VideoCapture('test.avi')
cap.set(15, 5)

ret, img = cap.read()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 30.0, (400,400))

while(ret == True):
    ret, img = cap.read()
    #img = cv2.equalizeHist(img)
    if not ret:
        break
    img = cv2.resize(img,(400,400))
    base = img.copy()

    img = perspective_transform(img)
    img = med_blur(img)
    img = hsv_thresh(img)
    
    #img = np.uint8(img)
    #img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

    cv2.imshow("post", img)
    cv2.imshow("base", base)
    out.write(img)
    if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
        break                        
                            # press q to quit

cap.release()
out.release()


cv2.waitKey(0)
cv2.destroyALLWINDOWS()