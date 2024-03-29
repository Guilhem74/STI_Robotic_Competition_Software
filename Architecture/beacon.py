import cv2
import numpy as np
import math
from picamera import PiCamera
import time
center_cone_x = 978#978 #974
center_cone_y = 655#655 #654
RedLed = (0,8040)
GreenLed = (8040,8040)
BlueLed = (8040,0)
YellowLed = (0,0)


g_low = (32, 110, 90)
g_high = (43, 255,255)
b_low = (10, 200, 75)
b_high = (15, 255,255)
r_low = (115, 105, 50)
r_high = (130, 255,255)


y_low = (55,50, 50)
y_high = (80, 255,255)


def center(selection,channel,img,center_cone_x,center_cone_y, gray_images):

    center_position = [(0,0),(0,0),(0,0)]
    color = [(0,0,255),(0,255,0),(255,0,0)]
    # convert the grayscale image to binary image
    for i in selection:
        gray_image = gray_images[i]#cv2.cvtColor(channel[i], cv2.COLOR_BGR2GRAY)
        gray_image = cv2.GaussianBlur(gray_image,(5,5),1)
        

        
        # find contours in the binary image
        _, contours, _ = cv2.findContours(gray_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        c = max(contours, key = cv2.contourArea)
        # calculate moments for each contour
        M = cv2.moments(c)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        center_position[i] = (cX,cY)
        end = time.time()
        print('Time to detect contour and center: ', end - start)
        cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
        cv2.line(img,(cX ,cY),(center_cone_x,center_cone_y),color[i], 2)
     
    cv2.line(img,(center_cone_x +200  ,center_cone_y  ),(center_cone_x,center_cone_y),[255,255,255], 1)
    

    cv2.circle(img, (center_cone_x, center_cone_y), 5, (255, 255, 255), -1)
    cv2.imwrite('photo_beacon_post.jpg',img )
    return center_position




def extract_channel(img):

    #img = cv2.GaussianBlur(img,(5,5),1)

    ## convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    mask_g = cv2.inRange(hsv,g_low ,g_high )
    mask_b = cv2.inRange(hsv, b_low, b_high)
    mask_r = cv2.inRange(hsv, r_low, r_high)
    #mask_y = cv2.inRange(hsv, y_low, y_high)
    
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


    



    ## slice the green
    imask_r = mask_r>0
    imask_g = mask_g>0
    imask_b = mask_b>0
    #imask_y = mask_y>0
  

    green = np.zeros_like(img, np.uint8)
    red = np.zeros_like(img, np.uint8)
    blue = np.zeros_like(img, np.uint8)
    #yellow = np.zeros_like(img, np.uint8)


    red[imask_r] = img[imask_r]
    green[imask_g] = img[imask_g]
    blue[imask_b] = img[imask_b]
    #yellow[imask_y] = img[imask_y]

    ## save 

    #image = np.zeros_like(img, np.uint8)
    #image = red + blue +green# + yellow
    #channel = [yellow, red,green,blue]
    channel = [ red,green,blue]
    
    return channel




def channel_selection(channel):
    
    selection = []
    position = []
    gray_images = []
    for i in range(len(channel)):
        
        
        gray_image = cv2.cvtColor(channel[i], cv2.COLOR_BGR2GRAY)
        gray_images.append(gray_image)
        non_zero = np.nonzero(gray_image)
        l = len(non_zero[0])//2
        if l > 0:
            selection.append(i)
            position = ((non_zero[1][l],non_zero[0][l]))
        
        
    #if len(selection) == 4 :
    #    try:
    #        selection.remove(r)
    #    except:
    #        pass
    return selection , gray_images#, position






    

def get_angles(selection, center_position,center_cone_x,center_cone_y ):
    vec = [[0,0],[0,0],[0,0]]
    angle = [0,0,0]
    
    ref_vector = (200,0) 
    
    ref_norm = math.sqrt(ref_vector[0]*ref_vector[0] + ref_vector[1]*ref_vector[1])
    for i in selection:

        vec[i] = ([center_position[i][0] - center_cone_x , center_position[i][1] - center_cone_y ])
    
        


   
        try:
            
            angle[i]=((math.atan2(ref_vector[1],ref_vector[0]) - math.atan2(vec[i][1],vec[i][0])))
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

    
    
    x1 = B[1][0] - B[0][0] 

    y1 = B[1][1] - B[0][1]


    x3 = B[2][0] - B[0][0]

    y3 = B[2][1] - B[0][1]

    x2 = B[0][0]

    y2 = B[0][1]


    t1 = 1/math.tan((angle[selection[0]] - angle[selection[1]] ))

    t2 = 1/math.tan((angle[selection[2]] - angle[selection[0]]  ))

    t3 = (1-t1*t2)/(t1+t2)

    x12 = x1 + t1*y1
    y12 = y1 - t1*x1

    x23 = x3 - t2*y3
    y23 = y3+t2*x3

    x31 = (x3+x1) + t3*(y3 - y1)
    y31 = (y3 + y1) - t3*(x3-x1)

    

    k31 = x1*x3 + y1*y3 + t3*(x1*y3 - x3*y1)

    D = (x12 - x23 )*(y23 - y31) -(y12 - y23)*(x23 - x31)

    X =  x2 + (k31*(y12 - y23))/D
    Y = y2 + (k31*(x23 - x12))/D

    theta = math.atan2(y2 - Y , x2 - X)  - (angle[selection[0]])
    return X,Y , math.degrees(theta)


def beacon_main(cam):
    #Grey Line is front of the robot
    #Green LED is the reference

    
    Computed_position = []
    Valid_position = []
    #Takes picture of cone
    
    
    raw_image = np.empty((1088,1920,3),dtype = np.uint8)
    cam.capture(raw_image, 'rgb')
    raw_image = raw_image[:1080,:1920,:]
    
    
    #Flip image to get as a map view
    raw_image  = cv2.flip( raw_image, 1 )
    raw_image = cv2.cvtColor(raw_image,cv2.COLOR_RGB2BGR)
    
    cv2.imwrite('photo_beacon.jpg',raw_image )
     
    height,width,depth = raw_image.shape
    
    circle_img  = np.zeros((height,width), np.uint8)
    
    #Masks of cercles to keeps only exterior or cone
    cv2.circle(circle_img,(center_cone_x,center_cone_y),132,(255,255,255),thickness=-1)
    cv2.circle(circle_img,(center_cone_x,center_cone_y),95,(0,0,0),thickness=-1)
    imask = circle_img>0
    
    #Reconstructing masked picture
    picture = np.zeros_like(raw_image, np.uint8)
    picture[imask] = raw_image[imask]
    
    
    #Extract channels from the picture
    channel = extract_channel(picture)
    
    selection, gray_images = channel_selection(channel)
    print(selection)
    if len(selection) < 3:
        return []
        #Computed_position.append([-8000,-8000,0])
    else:

        position  = center(selection,channel,raw_image,center_cone_x,center_cone_y,gray_images)


        #display(channel,position,img,center_cone_x,center_cone_y , selection)


        angle = get_angles(selection, position,center_cone_x,center_cone_y )
        X ,Y ,A = get_position(selection,angle,[RedLed,GreenLed,BlueLed])
        return X,Y,A
    """
    for i in Computed_position:
        if ( i[0]>0 and i[0]< 7999 and i[1]>0 and i[1] <7999):
            Valid_position.append(i)


    distance = []

    for i in range(len(Valid_position)):
        dist = 0
        for j in range(len(Valid_position)):
            dist = dist +math.sqrt(abs(Valid_position[i][0] - Valid_position[j][0])*abs(Valid_position[i][0] - Valid_position[j][0])  + abs(Valid_position[i][1] - Valid_position[j][1])*abs(Valid_position[i][1] - Valid_position[j][1]))

        distance.append(dist)
    if len(distance )>0:  
        ret_index = np.argmin(distance)

        #cv2.imwrite('photo_beacon_post.jpg', img)
    """
    #return Valid_position[ret_index]
    #else:
        #return []
                    

    







    
    
    
    
