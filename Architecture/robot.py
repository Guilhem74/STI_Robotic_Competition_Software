
import numpy as np
import math

class robot:
    def __init__(self,x,y,a):
        self.x = int(round(x/10))
        self.y = int(round(y/10))
        self.angle = math.radians(a)

        self.range_ir = 10
        self.size_front = 30
        self.size_back = 10
        self.size_side = 20
        self.diag_front = math.sqrt(self.size_front* self.size_front + self.size_side*self.size_side)
        self.sensor_List = []
        self.ir_sensors = {
                           'FRONT':[ round((self.size_front + self.range_ir) * math.cos(self.angle))  ,
                            round((self.size_front + self.range_ir) * math.sin(self.angle))   , 0],
                           'BACK':[round((self.size_back + self.range_ir) * math.cos(self.angle+math.pi)) , 
                           round((self.size_back + self.range_ir) * math.sin(self.angle+math.pi))  , 0],
                           'LEFT':[round( (self.size_side + self.range_ir) * math.cos(self.angle + math.pi/2))  ,
                            round( (self.size_side + self.range_ir) * math.sin(self.angle + math.pi/2)) , 0],
                           'RIGHT':[round((self.size_side + self.range_ir)*math.cos(self.angle - math.pi/2)) , 
                           round((self.size_side + self.range_ir)*math.sin(self.angle - math.pi/2))   , 0],
                           'FRONTLEFT':[round((self.diag_front + self.range_ir)*math.cos(self.angle + math.pi/4)) , 
                                               round((self.diag_front + self.range_ir)*math.sin(self.angle + math.pi/4))   , 0],

                           'FRONTRIGHT':[round((self.diag_front + self.range_ir)*math.cos(self.angle - math.pi/4)) , 
                                               round((self.diag_front + self.range_ir)*math.sin(self.angle - math.pi/4))   , 0],}


    def set_position(self, x,y,a):
        self.x = int(round(x/10))
        self.y = int(round(y/10))
        self.angle = math.radians(a)
        self.ir_sensors = {
                           'FRONT':[ round((self.size_front + self.range_ir) * math.cos(self.angle))  ,
                            round((self.size_front + self.range_ir) * math.sin(self.angle))   , 0],
                           'BACK':[round((self.size_back + self.range_ir) * math.cos(self.angle+math.pi)) , 
                           round((self.size_back + self.range_ir) * math.sin(self.angle+math.pi))  , 0],
                           'LEFT':[round( (self.size_side + self.range_ir) * math.cos(self.angle + math.pi/2))  ,
                            round( (self.size_side + self.range_ir) * math.sin(self.angle + math.pi/2)) , 0],
                           'RIGHT':[round((self.size_side + self.range_ir)*math.cos(self.angle - math.pi/2)) , 
                           round((self.size_side + self.range_ir)*math.sin(self.angle - math.pi/2))   , 0],
                           'FRONTLEFT':[round((self.diag_front + self.range_ir)*math.cos(self.angle + math.pi/4)) , 
                                               round((self.diag_front + self.range_ir)*math.sin(self.angle + math.pi/4))   , 0],
                           'FRONTRIGHT':[round((self.diag_front + self.range_ir)*math.cos(self.angle - math.pi/4)) , 
                                               round((self.diag_front + self.range_ir)*math.sin(self.angle - math.pi/4))   , 0],}
    def get_position(self):
        return [self.x*10,self.y*10,self.angle]




    def get_beacon_position(self):
        return beacon_main()

    def rotation(self,x,y,cx,cy,theta):
        tempX = x - cx
        tempY = y - cy

        #now apply rotation
        rotatedX = tempX*math.cos(theta) - tempY*math.sin(theta)
        rotatedY = tempX*math.sin(theta) + tempY*math.cos(theta)

        #translate back
        new_x = round(rotatedX + cx)
        new_y = round(rotatedY + cy)
        return new_x,new_y

    def sensor_state(self,state):
        obstacle_position = []
        sensors_state = {'a':1,'b':2,'c':4,'d':8,'e':16,'f':32,'g':64,'h':128,'i':256,'j':512,'k':1024,'l':2048,'m':4096}

        mask = 1 
        for sensor in  sensors_state:
            if state&(mask*sensors_state[sensor])>0:
                sensors_state[sensor]=1
                print(sensor)

            else:
                sensors_state[sensor]=0
        print(sensors_state)

        #Front 
        if ( sensors_state['m'] ==1 or (sensors_state['l'] == 1 and  sensors_state['e'] ==0) or (sensors_state['k'] ==1  and sensors_state['g'] == 0 ) ) :

            xi = self.ir_sensors["FRONT"][0] + self.x
            yi = self.ir_sensors["FRONT"][1] + self.y
            obstacle_position.append([xi,yi])
            print('Front')
        #BACK
        if sensors_state['a']  or sensors_state['b']   or sensors_state['c'] or sensors_state['b']  :
            xi = self.ir_sensors["BACK"][0] + self.x
            yi = self.ir_sensors["BACK"][1] + self.y
            obstacle_position.append([xi,yi])
            print('Back')
        #LEFT
        if ( sensors_state['h'] == 1  or sensors_state['j'] == 1 or (  sensors_state['g'] == 1 and sensors_state['k'] == 0)):
            xi = self.ir_sensors["LEFT"][0] + self.x
            yi = self.ir_sensors["LEFT"][1] + self.y
            obstacle_position.append([xi,yi])
            print('Left')

        #RIGHT
        if ( sensors_state['f'] == 1  or sensors_state['i'] == 1 or (  sensors_state['e'] == 1 and sensors_state['l'] == 0)):
            xi = self.ir_sensors["RIGHT"][0] + self.x
            yi = self.ir_sensors["RIGHT"][1] + self.y
            obstacle_position.append([xi,yi])
            print('Right')
        #FRONT LEFT
        if sensors_state['g'] and  sensors_state['k'] :
            xi = self.ir_sensors["FRONTLEFT"][0] + self.x
            yi = self.ir_sensors["FRONTLEFT"][1] + self.y
            obstacle_position.append([xi,yi])
            print('FrontL')
        #FRONT RIGH
        if sensors_state['e'] and sensors_state['l']:
            xi = self.ir_sensors["FRONTRIGHT"][0] + self.x
            yi = self.ir_sensors["FRONTRIGHT"][1] + self.y
            obstacle_position.append([xi,yi])
            print('FrontR')



        return obstacle_position
        
    def Has_reached_final_position(self, Final_Pos):
        #Coordinate must be in mm
        X,Y,_=self.get_position()
        Distance_X=Final_Pos[0]-X
        Distance_Y=Final_Pos[1]-Y
        if(math.sqrt(Distance_X*Distance_X+Distance_Y*Distance_Y)<100):
            return True
        return False
    
    
    #Update corner robot
   # for i in corner:
    #  x = i[0]
    #  y = i[1]
    #  x,y = self.rotation(x,y,cx,cy,theta)
   #  grid[x,y] = [255,255,255]
    #Update sensor position
  #  for s in self.sensor_List:
  #    s.set_position(self)


