# Import essential libraries
import requests
import cv2
import numpy as np
import imutils
import numpy as np
import cv2
import cv2.aruco as aruco
import math

url = "http://192.168.167.180:8080/shot.jpg"
global id,center_x,center_y, angle
corner = [0]*5

def aruco_pixel_coordinates(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow(gray)
    aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    
    if ids !=None :
        id=int(ids)
        corner[0]=(corners[0][0][0])
        corner[1]=(corners[0][0][1])
        corner[2]=(corners[0][0][2])
        corner[3]=(corners[0][0][3])

        # print(id,corner)
        center_x=(corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4
        center_y=(corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4

        top_center_x=(corner[0][0]+corner[1][0])/2
        top_center_y=(corner[0][1]+corner[1][1])/2

        x2 = top_center_x-center_x
        y2 = top_center_y-center_y

        if top_center_y<center_y:
            theta = math.atan2(-y2, x2)
            angle = math.degrees(theta) 
        else:
            theta = math.atan2(-y2, x2)
            angle = 360+math.degrees(theta)

        if top_center_y == center_y and center_x > top_center_x:
            angle = 180.0
        elif top_center_y == center_y and center_x < top_center_x:
            angle = 0.0	
        elif top_center_x == center_x and top_center_y < center_y:
            angle = 90.0
        elif top_center_x == center_x and top_center_y > center_y:
            angle = 270.0

        print(id,center_x,center_y,angle)

        center_x=int((corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4)
        center_y=int((corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4)
        top_center_x=int((corner[0][0]+corner[1][0])/2)
        top_center_y=int((corner[0][1]+corner[1][1])/2)

        cv2.circle(img,(center_x,center_y),5,(0,0,255),-1)
        cv2.circle(img,(int(corner[0][0]),int(corner[0][1])), 5, (125,125,125), -1)
        cv2.circle(img,(int(corner[1][0]),int(corner[1][1])), 5, (0,255,0), -1)
        cv2.circle(img,(int(corner[2][0]),int(corner[2][1])), 5, (180,105,255), -1)
        cv2.circle(img,(int(corner[3][0]),int(corner[3][1])), 5, (255,255,255), -1)
        cv2.line(img,(center_x,center_y),(top_center_x,top_center_y),(255,0,0),3)

        cv2.putText(img,str(id),(center_x+20,center_y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),3,cv2.LINE_AA)
        cv2.putText(img,str(int(angle)),(center_x-90,center_y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,255,0),3,cv2.LINE_AA)
        cv2.imshow("Prepocessed", img)


while True:
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = imutils.resize(img, width=1000, height=1800)
    # cv2.imshow("Image from IP Webcam", img)
    aruco_pixel_coordinates(img)

    if cv2.waitKey(1) == 27:
        break
  
cv2.destroyAllWindows()
