
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
                       'FRONTLEFT':[round((self.diag_front + self.range_ir)*math.cos(self.angle - math.pi/4)) , 
                                           round((self.diag_front + self.range_ir)*math.sin(self.angle - math.pi/4))   , 0],
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
                       'FRONTLEFT':[round((self.diag_front + self.range_ir)*math.cos(self.angle - math.pi/4)) , 
                                           round((self.diag_front + self.range_ir)*math.sin(self.angle - math.pi/4))   , 0],
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
   
  def sensor_state(self, state):
      obstacle_position = []
      sensors_state = {'a':0,'b':0,'c':0,'d':0,'e':0,'f':0,'g':0,'h':0,'i':0,'j':0,'k':0,'l':0,'m':0}
        
      mask = 1 
      for sensor in  sensors_state:
          if state&mask>0:
              sensors_state[sensor]=1
          mask = mask * 2
          print(sensors_state)
      #Front 
      if sensors_state['k'] +sensors_state['m']  + sensors_state['l'] >0:
            
            xi = robot.ir_sensors["FRONT"][0] + self.x
            yi = robot.ir_sensors["FRONT"][1] + self.y
            obstacle_position.apppend([xi,yi])
      #BACK
      if sensors_state['a'] +sensors_state['b']  + sensors_state['c'] +sensors_state['b']  >0:
            xi = robot.ir_sensors["BACK"][0] + self.x
            yi = robot.ir_sensors["BACK"][1] + self.y
            obstacle_position.apppend([xi,yi])
      #LEFT
      if sensors_state['h'] +sensors_state['j']  + sensors_state['g']    >0:
            xi = robot.ir_sensors["LEFT"][0] + self.x
            yi = robot.ir_sensors["LEFT"][1] + self.y
            obstacle_position.apppend([xi,yi])
      #RIGHT
      if sensors_state['e'] +sensors_state['i']  + sensors_state['f']    >0:
            xi = robot.ir_sensors["RIGHT"][0] + self.x
            yi = robot.ir_sensors["RIGHT"][1] + self.y
            obstacle_position.apppend([xi,yi])
      #FRONT LEFT
      if sensors_state['g'] + sensors_state['k'] == 2:
            xi = robot.ir_sensors["FRONTLEFT"][0] + self.x
            yi = robot.ir_sensors["FRONTLEFT"][1] + self.y
            obstacle_position.apppend([xi,yi])
      #FRONT RIGH
      if sensors_state['e'] + sensors_state['l'] == 2:
            xi = robot.ir_sensors["FRONTRIGHT"][0] + self.x
            yi = robot.ir_sensors["FRONTRIGHT"][1] + self.y
            obstacle_position.apppend([xi,yi])
        
        

      return obstacle_position
        
    
    
    
    #Update corner robot
   # for i in corner:
    #  x = i[0]
    #  y = i[1]
    #  x,y = self.rotation(x,y,cx,cy,theta)
   #  grid[x,y] = [255,255,255]
    #Update sensor position
  #  for s in self.sensor_List:
  #    s.set_position(self)


