
import numpy as np
from PIL import Image
from heapq import *
import math
import threading
from collections import deque
from sensor import *



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
    for sensor in self.ir_sensors:
      self.sensor_List.append((self,sensor,self.ir_sensors[sensor][0],self.ir_sensors[sensor][1],self.ir_sensors[sensor][2]))




    self.color = [255, 255, 0]
  def set_position(self, x,y,a):
    self.x = x
    self.y = y
    self.a = a
    

  def get_position(self):
    return [self.x, self.y, self.a]
    
  def display(self,grid):
    theta = robot.angle
    cx = robot.x
    cy = robot.y
    #Front left corner of robot
    x0 = cx - robot.size_front 
    y0 = cy - robot.size_side 
    #Front right corner of robot
    x1 = cx - robot.size_front 
    y1 = cy + robot.size_side 
    #back left corner of robot
    x2 = cx + robot.size_back 
    y2 = cy - robot.size_side 
    #Back right corner of robot
    x3 = cx + robot.size_front
    y3 = cy + robot.size_side

    corner = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]

    for i in corner:
      x = i[0]
      y = i[1]

      tempX = x - cx
      tempY = y - cy

      #now apply rotation
      rotatedX = tempX*cos(theta) - tempY*sin(theta)
      rotatedY = tempX*sin(theta) + tempY*cos(theta)

      #translate back
      i[0] = rotatedX + cx
      i[1] = rotatedY + cy
