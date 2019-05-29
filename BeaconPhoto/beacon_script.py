import cv2
import numpy as np
import math




center_cone_x =  370
center_cone_y = 262
RedLed = (0,8000)
GreenLed = (8000,8000)
BlueLed = (8000,0)
YellowLed = (0,0)

g_low = (30, 70, 75)
g_high = (50, 255,255)
b_low = (0, 50, 50)
b_high = (0, 150,150)
r_low = (110, 100, 100)
r_high = (130, 255,255)

y_low = (80,50, 50)
y_high = (90, 255,255)

def extract_channel(img):

    img = cv2.GaussianBlur(img,(5,5),1)

    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    mask_g = cv2.inRange(hsv,g_low ,g_high )
    mask_b = cv2.inRange(hsv, b_low, b_high)
    mask_r = cv2.inRange(hsv, r_low, r_high)
    mask_y = cv2.inRange(hsv, y_low, y_high)
    
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    



    ## slice the green
    imask_r = mask_r>0
    imask_g = mask_g>0
    imask_b = mask_b>0
    imask_y = mask_y>0
  

    green = np.zeros_like(img, np.uint8)
    red = np.zeros_like(img, np.uint8)
    blue = np.zeros_like(img, np.uint8)
    yellow = np.zeros_like(img, np.uint8)


    red[imask_r] = img[imask_r]
    green[imask_g] = img[imask_g]
    blue[imask_b] = img[imask_b]
    yellow[imask_y] = img[imask_y]

    ## save 

    image = np.zeros_like(img, np.uint8)
    image = red + blue +green + yellow
    channel = [yellow, red,green,blue]
    return channel , image




def channel_selection(channel):
    color = [(0,255,255),(255,0,0,),(0,255,0),(0,0,255)]
    selection = []
    position = []
    for i in range(len(channel)):
        
        gray_image = cv2.cvtColor(channel[i], cv2.COLOR_BGR2GRAY)
        non_zero = np.nonzero(gray_image)
        l = len(non_zero[0])//2
        if l > 0:
            selection.append(i)
            position.append((non_zero[1][l],non_zero[0][l]))
        
        
        
    if len(selection) == 4 :
        selection.pop()

    return selection , position






def display(channel, position, img, center_cone_x, center_cone_y, selection):

    
    color = [(0,255,255),(0,0,255),(0,255,0),(255,0,0)]
    for i in selection:
        cv2.circle(img, position[i], 5, (255, 255, 255), -1)
        cv2.line(img,position[i],(center_cone_x,center_cone_y),color[i], 2)
     
    cv2.line(img,(center_cone_x +200  ,center_cone_y  ),(center_cone_x,center_cone_y),[255,255,255], 1)
    

    cv2.circle(img, (center_cone_x, center_cone_y), 5, (255, 255, 255), -1)
    cv2.imshow('image',img)
    cv2.waitKey(0)
    

def get_angles(selection, center_position,center_cone_x,center_cone_y ):
    vec = []
    angle = []
    norm = []
    ref_vector = (200,0) 
    print(center_position)
    ref_norm = math.sqrt(ref_vector[0]*ref_vector[0] + ref_vector[1]*ref_vector[1])
    for i in selection:

        vec.append([center_position[i][0] - center_cone_x,center_position[i][1] - center_cone_y ])
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


def get_position(selection,angle,Beacon_position):
    #Choose Balise GreenLed as origin
    B = []
    for i in selection:
        B.append(Beacon_position[i])

    print(angle)
    x1 = B[1][0] - B[0][0] 

    y1 = B[1][1] - B[0][1]


    x3 = B[2][0] - B[0][0]

    y3 = B[2][1] - B[0][1]

    x2 = B[0][0]

    y2 = B[0][1]


    t1 = 1/math.tan((angle[1] - angle[0] ))

    t2 = 1/math.tan((angle[2] - angle[1] ))

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
    

    #Led positions 
    

    raw_image = cv2.imread('image_beacon(35,35)_s=6000.jpg')
    raw_image  = cv2.flip( raw_image, 1 )



    height,width,depth = raw_image.shape
    circle_img = np.zeros((height,width), np.uint8)
    cv2.circle(circle_img,(center_cone_x,center_cone_y),500,(0,0,0),thickness=-1)
    cv2.circle(circle_img,(center_cone_x,center_cone_y),140,(255,255,255),thickness=-1)
    cv2.circle(circle_img,(center_cone_x,center_cone_y),120,(0,0,0),thickness=-1)
    imask = circle_img>0
    picture = np.zeros_like(raw_image, np.uint8)
    picture[imask] = raw_image[imask]
    cv2.imshow("masked", picture)
    cv2.waitKey(0)


    
    channel,img = extract_channel(picture)

    selection, position = channel_selection(channel)
    if len(selection) < 3:
        return None

    cv2.imshow('image',img)
    cv2.waitKey(0)
    

    display(channel,position,img,center_cone_x,center_cone_y , selection)
   

    angle = get_angles(selection, position,center_cone_x,center_cone_y )
    


    return get_position(selection,angle,[YellowLed,RedLed,GreenLed,BlueLed])




print(beacon_main())


    
    
    
    
