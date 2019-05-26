import cv2
import numpy as np
import math
from picamera import PiCamera


#Led positions 
center_cone_x =  369
center_cone_y = 262
RedLed = (0,0)
GreenLed = (3000,3000)
BlueLed = (0,3000)
YellowLed = (3000,0)

g_low = (36, 75, 75)
g_high = (70, 255,255)
b_low = (0, 100, 100)
b_high = (15, 255,255)
r_low = (105, 75, 75)
r_high = (130, 255,255)

g_low = (0, 0, 0)
g_high = (255, 255,255)
b_low = (0, 0, 0)
b_high = (255, 255,255)
r_low = (0, 0, 0)
r_high = (255, 255,255)

def extract_channel(img):

    img = cv2.GaussianBlur(img,(5,5),0)

    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    mask_g = cv2.inRange(hsv,g_low ,g_high )
    mask_b = cv2.inRange(hsv, b_low, b_high)
    mask_r = cv2.inRange(hsv, r_low, r_high)

    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    



    ## slice the green
    imask_r = mask_r>0
    imask_g = mask_g>0
    imask_b = mask_b>0
  

    green = np.zeros_like(img, np.uint8)
    red = np.zeros_like(img, np.uint8)
    blue = np.zeros_like(img, np.uint8)


    red[imask_r] = img[imask_r]
    green[imask_g] = img[imask_g]
    blue[imask_b] = img[imask_b]

    ## save 

    image = np.zeros_like(img, np.uint8)
    image = red + blue +green
    channel = [red,green,blue]
    return channel , image








def center(channel,img,center_cone_x,center_cone_y):

    center_position = []
    color = [(0,0,255,),(0,255,0),(255,0,0)]
    # convert the grayscale image to binary image
    for i in range(len(channel)):
        gray = cv2.cvtColor(channel[i], cv2.COLOR_BGR2GRAY)

        gray_image = cv2.GaussianBlur(gray,(5,5),0) 
        # find contours in the binary image
        _, contours, _ = cv2.findContours(gray_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours) == 0):
            return None, None
        c = contours[0]
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except:
            cX = 0
            cY = 0
            
        center_position.append([cX,cY])
        cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
        cv2.line(img,(cX ,cY),(center_cone_x,center_cone_y),color[i], 2)
     
    cv2.line(img,(center_cone_x +200  ,center_cone_y  ),(center_cone_x,center_cone_y),[255,255,255], 1)
    

    cv2.circle(img, (center_cone_x, center_cone_y), 5, (255, 255, 255), -1)
    return center_position, img

def get_angles(center_position,center_cone_x,center_cone_y ):
    vec = []
    angle = []
    norm = []
    ref_vector = (200,0) 
    ref_norm = math.sqrt(ref_vector[0]*ref_vector[0] + ref_vector[1]*ref_vector[1])
    for i in center_position:

        vec.append([i[0] - center_cone_x,i[1] - center_cone_y ])
    for i in vec:
        norm.append( math.sqrt(i[0]*i[0] + i[1]*i[1]) ) 


    for i in range(len(vec)):
        try:
            
            angle.append((math.atan2(ref_vector[1],ref_vector[0]) - math.atan2(vec[i][1],vec[i][0])))
        except:
            pass



    angle = np.array(angle)
    
    #angle = angle[np.nonzero(angle)]

    #RG RB GB


    return angle


def get_position(angle,RedLed,GreenLed,BlueLed):
    #Choose Balise GreenLed as origin
    x1 = RedLed[0] - GreenLed[0] 

    y1 = RedLed[1] - GreenLed[1]

    x3 = BlueLed[0] - GreenLed[0]

    y3 = BlueLed[1] - GreenLed[1]

    x2 = GreenLed[0]

    y2 = GreenLed[1]

    try:
        t1 = 1/math.tan((angle[1] - angle[0] ))

        t2 = 1/math.tan((angle[2] - angle[1] ))
    except:
        t1 = 1
        t2 = 2

    t3 = (1-t1*t2)/(t1+t2)

    x12 = x1 + t1*y1
    y12 = y1 - t1*x1

    x23 = x3 + t2*y3
    y23 = y3+t2*x3

    x31 = (x3+x1) + t3*(y3 - y1)
    y31 = (y3 + y1) - t3*(x3-x1)

    

    k31 = x1*x3 + y1*y3 + t3*(x1*y3 - x3*y1)

    D = (x12 - x23 )*(y23 - y31) -(y12 - y23)*(x23 - x31)

    X =  x2 + (k31*(y12 - y23))/D
    Y = y2 + (k31*(x23 - x12))/D

    theta = math.atan2(y2 - Y , x2 - X)  - (angle[1])
    return X,Y , theta




def beacon_main():
    #Grey Line is front of the robot
    #Green LED is the reference
    

    
    camera = PiCamera()
    camera.shutter_speed = 8000
    camera.capture('image_beacon.jpg')
    raw_image = cv2.imread('image_beacon.jpg')

    camera.close()
    raw_image  = cv2.flip( raw_image, 1 )
    
    channel,img = extract_channel(raw_image)

    

    center_position, r_img = center(channel,raw_image,center_cone_x,center_cone_y)
   
    if center_position == None:
        return None
    angle = get_angles(center_position,center_cone_x,center_cone_y )
    cv2.imwrite('angle.jpg',r_img)
    

    return get_position(angle,RedLed,GreenLed,BlueLed)




#print(beacon_main())


    
    
    
    
