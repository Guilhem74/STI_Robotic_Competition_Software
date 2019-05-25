
import numpy as np
import math
import beacon as Beacon
class robot:
    def __init__(self,x,y,a):
        self.x = int(round(x/10))
        self.y = int(round(y/10))
        self.angle = math.radians(a)

        
        
        self.size_front = 33
        self.size_back = 7
        self.size_side = 20
        self.diag_front = math.sqrt(self.size_front* self.size_front + self.size_side*self.size_side)
        self.sensor_List = []
        self.ir_sensors_position = {'a':[84/10,133.4,160,20],
                           'b':[171/10,107,-135,20],
                           'c':[171/10,-107,135,20],
                           'd':[84/10,-133.4,-160,20],
                           'e':[287/10,-41,-15,20],
                           'f':[186/10,-95.8,-20,20],
                           'g':[287/10,41,15,20],
                           'h':[186/10,95.8,20,20],
                           'i':[369/10,-31.91,-130,20],
                           'j':[369/10,31.91,130,20],
                           'k':[333/10,-7,-55,20],
                           'l':[333/10,7,55,20],
                           'm':[340/10,0,0,20],}
        
                           
        self.ir_sensors = {
            'a':[0,0  ],
            'b':[0,0  ],
            'c':[0,0  ],
            'd':[0,0  ],
            'e':[0,0  ],
            'f':[0,0  ],
            'g':[0,0  ],
            'h':[0,0  ],
            'i':[0,0  ],
            'j':[0,0  ],
            'k':[0,0  ],
            'l':[0,0  ],
            'm':[0,0  ],}
        for sensor in self.ir_sensors:
            self.sensor_position(sensor,self.ir_sensors_position[sensor][0],self.ir_sensors_position[sensor][1],self.ir_sensors_position[sensor][2] )
            
    
    
    
    def sensor_position(self, sensor_id, dist,theta,angle_sensor ):
        #dist = math.sqrt(x*x+y*y)
        theta = math.radians(theta)
        angle_sensor = math.radians(angle_sensor)
        #ref_vector = (math.cos(self.angle),math.sin(self.angle))
        #v2 = (x,y)
        #theta = self.anglevec(ref_vector, v2)
        #if sensor_id in ['c','d','f','e','k','i']:
         #   theta = - theta
        #theta = math.atan2(ref_vector[1],ref_vector[0]) - math.atan2(y,x)
        
       
        self.ir_sensors[sensor_id] = [round(dist*math.cos(self.angle + theta) + (self.ir_sensors_position[sensor_id][3] * math.cos(self.angle +angle_sensor))),
                                     round(dist*math.sin(self.angle + theta) + (self.ir_sensors_position[sensor_id][3]  *math.sin(self.angle +angle_sensor))),]

        
          
    
    def set_position(self, x,y,a):
        self.x = int(round(x/10))
        self.y = int(round(y/10))
        self.angle = math.radians(a)
        for sensor in self.ir_sensors:
            self.sensor_position(sensor,self.ir_sensors_position[sensor][0],self.ir_sensors_position[sensor][1],self.ir_sensors_position[sensor][2]  )
        
        
        
    def get_position(self):
        return [self.x*10,self.y*10,self.angle]



    def dotproduct(self,v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

    def length(self,v):
        return math.sqrt(self.dotproduct(v, v))

    def anglevec(self,v1, v2):
        return math.acos(self.dotproduct(v1, v2) / (self.length(v1) * self.length(v2)))


    def get_beacon_position(self):
        return Beacon.beacon_main()

    

    def sensor_state(self,state):
        obstacle_position = []
        sensors_state = {'a':1,'b':2,'c':4,'d':8,'e':16,'f':32,'g':64,'h':128,'i':256,'j':512,'k':1024,'l':2048,'m':4096}

        mask = 1 
        for sensor in  sensors_state:
            if state&(mask*sensors_state[sensor])>0:
                sensors_state[sensor]=1
                

            else:
                sensors_state[sensor]=0
        print(sensors_state)

        
        for i in sensors_state :
            if sensors_state[i] :

                xi = self.ir_sensors[i][0] + self.x
                yi = self.ir_sensors[i][1] + self.y
                obstacle_position.append([xi,yi])
                
        



        return obstacle_position
        
    def Has_reached_final_position(self, Final_Pos):
        #Coordinate must be in mm
        X,Y,_=self.get_position()
        Distance_X=Final_Pos[0]-X
        Distance_Y=Final_Pos[1]-Y
        if(math.sqrt(Distance_X*Distance_X+Distance_Y*Distance_Y)<200):
            return True
        return False
    
    




