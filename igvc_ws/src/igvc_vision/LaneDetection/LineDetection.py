import cv2
import numpy as np
def perspective_transform(img, h, d1, w1, d2, w2,fx,fy):
    height = h
    closedist = d1
    closewidth = w1
    middist = d2
    midwidth = w2
    per_inch = 1
    map_height = fy
    map_width = fx
    trim = int(((float(midwidth - closewidth)/2)/float(middist - closedist))*map_height)
    rectangle_height = int(.88*map_height)#number is ratio for how far down the frame rectangle should start




    pts_src = np.float32([[0, int(map_height)], [map_width, int(map_height)], [0, 0], [map_width, 0]])
    left_trim = trim
    right_trim = map_width - trim
    pts_dst = np.float32([[left_trim, map_height],[right_trim, map_height],[0, 0],[map_width,0]])
    mtx = cv2.getPerspectiveTransform(pts_src,pts_dst)
    flat_map = cv2.warpPerspective(img, mtx, (map_width, map_height))
    cv2.rectangle(flat_map, (60+left_trim,rectangle_height), (right_trim-60,map_height), (0,0,0), -1)#occlude bender chassis
    return flat_map
    
def med_blur(img):
    img = cv2.medianBlur(img,7)
    return img
def hsv_thresh(img):
    
    #Convert to hsv
    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #split HSV into individual componenets
    #h, s, v = cv2.split(frame_HSV)
    #equalize Histogram to account for lighting and remerge after equalizing
    #v = cv2.equalizeHist(v)
    #frame_HSV = cv2.merge([h,s,v])
    #filters out everything except the line
    #frame_threshold = cv2.inRange(frame_HSV, (25, 30, 80), (40, 100, 225))
    frame_threshold = cv2.inRange(frame_HSV, (30, 30, 90), (40, 100, 210))
    #frame_threshold = 1 - frame_threshold
    return frame_threshold


#img = cv2.imread("test1.jpg")
cap = cv2.VideoCapture('test3.avi')
cap.set(15, -10)

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
    #img = perspective_transform(img,53,38,76,91,134, 400, 400)
    img = med_blur(img)
    img = hsv_thresh(img)
    img = cv2.GaussianBlur(img, (17,17),0)
    
    #contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    #cv2.drawContours(img, contours, -1, (0,255,0), 3)
    #img, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #img = np.uint8(img)
    #img = cv2.cvtColor(img, cv2.COLOR_HSV2GRAY)
    #img = cv2.Canny(img,100,200)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
    #img = 1 - img
    img2 = cv2.cvtColor(img.copy(),cv2.COLOR_GRAY2BGR)
    img2 = cv2.drawContours(img2, output, -1, (0,255,0), 3)
    cv2.imshow("post", img2)
    cv2.imshow("base", base)
    out.write(img)
    if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
        break                        
                            # press q to quit

cap.release()
out.release()


cv2.waitKey(0)
cv2.destroyALLWINDOWS()