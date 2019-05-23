
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
    self.sensor_List = []
    self.ir_sensors = {
                       'FRONT':[ round((self.size_front + self.range_ir) * math.cos(self.angle))  ,
                        round((self.size_front + self.range_ir) * math.sin(self.angle))   , 0],
                       'BACK':[round((self.size_back + self.range_ir) * math.cos(self.angle+math.pi)) , 
                       round((self.size_back + self.range_ir) * math.sin(self.angle+math.pi))  , 0],
                       'LEFT':[round( (self.size_side + self.range_ir) * math.cos(self.angle + math.pi/2))  ,
                        round( (self.size_side + self.range_ir) * math.sin(self.angle + math.pi/2)) , 0],
                       'RIGHT':[round((self.size_side + self.range_ir)*math.cos(self.angle - math.pi/2)) , 
                       round((self.size_side + self.range_ir)*math.sin(self.angle - math.pi/2))   , 0],} #Sensor position
    



    
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
                       round((self.size_side + self.range_ir)*math.sin(self.angle - math.pi/2))   , 0],}
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


    #Update corner robot
   # for i in corner:
    #  x = i[0]
    #  y = i[1]
    #  x,y = self.rotation(x,y,cx,cy,theta)
   #  grid[x,y] = [255,255,255]
    #Update sensor position
  #  for s in self.sensor_List:
  #    s.set_position(self)


