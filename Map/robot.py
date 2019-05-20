
import numpy as np
from PIL import Image
from heapq import *
import math
import threading
from collections import deque
from map import *
from robot import *
from pathfinder import *
from case import *
from sensor import *
from beacon import *

class robot:
  def __init__(self,x,y,a):
    self.x = x
    self.y = y
    self.angle = a
    self.size_front = 7
    self.size_back = 2
    self.size_side = 7
    self.sensor_List = []
    self.ir_sensors = {'A':[-self.size_front  , (15/40)*self.size_side ,  0],
                       'B':[-self.size_front  , 0  , 0],
                       'C':[-self.size_front  , -(15/40)*self.size_side  , 0],
                       'D':[-(2/3)*self.size_front  , -self.size_side  , 0],
                       'E':[-0.15*self.size_front ,  -self.size_side ,  0],
                       'F':[self.size_back  , -0.8*self.size_side  , 0],
                       'G':[self.size_back  , -0.33*self.size_side ,  0],
                       'H':[self.size_back  , 0.33*self.size_side  , 0],
                       'I':[self.size_back  , 0.8*self.size_side ,  0],
                       'J':[-0.15*self.size_front ,  self.size_side ,  0],
                       'K':[-(2/3)*self.size_front  , self.size_side  , 0],
                       'L':[-self.size_front  , 0  , 0],
                       'M':[-self.size_front  , 0  , 0],} #Sensor position
    for s in self.ir_sensors:
      self.sensor_List.append(sensor(s,self.ir_sensors[s][0],self.ir_sensors[s][1],self.ir_sensors[s][2]))
    



    self.color = [255, 255, 0]
  def set_position(self, x,y,a):
    self.x = x
    self.y = y
    self.a = a

    

  def get_position(self):
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

  def display(self,grid,map):
    theta = self.angle
    cx = self.x
    cy = self.y
    #Front left corner of robot
    x0 = cx - self.size_front 
    y0 = cy - self.size_side 
    #Front right corner of robot
    x1 = cx - self.size_front 
    y1 = cy + self.size_side 
    #back left corner of robot
    x2 = cx + self.size_back 
    y2 = cy - self.size_side 
    #Back right corner of robot
    x3 = cx + self.size_back
    y3 = cy + self.size_side
    corner = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
    
    new_point = []

    
    #Update corner robot
    for i in corner:
      x = i[0]
      y = i[1]
      x,y = self.rotation(x,y,cx,cy,theta)
      
      grid[x,y] = case(map.def_zone(i[0],i[1]) , status = "robot" )
    #Update sensor position
    for s in self.sensor_List:
      s.set_position(self)
    for i in self.sensor_List:
      i.set_position(self)
      i.display(grid,map)



    return grid