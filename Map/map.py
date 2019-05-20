#Map generator



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
from map_items import *



class Map:
  def __init__(self):

    self.resolution = 10
    self.bottle = []
    self.obstacle = []
    self.size_robot = 1 * self.resolution
    self.H = 16*self.resolution
    self.W = 16*self.resolution
    self.walls = []
    #self.data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    self.map = np.zeros((self.H,self.W), dtype=int)
    self.grid = np.empty((self.H+20,self.W+20), dtype=object)
    self.build_map()



  def build_map(self):
    
    self.grid[:] = case(zone = 1 , status = "unexplored" )
    for i in range(self.H):
      for j in range(self.W):
        
        self.grid[i,j] = case(self.def_zone(i,j) , status = "unexplored" )
        if(i==0 or i==self.H-1 or j ==0 or j ==self.W-1):
          self.walls.append( wall(i,j,self,self.grid) )
          #self.grid[i,j] = case(self.def_zone(i,j)  , status = "wall" )


    for i in range(6*self.resolution):
      #self.grid[i,10*self.resolution] = case(self.def_zone(i,j)  , status = "wall" )
      self.walls.append(wall(i,10*self.resolution,self,self.grid))

    for i in range(5*self.resolution):
      #self.grid[6*self.resolution,10*self.resolution+i] = case(self.def_zone(i,j)  , status = "wall" )
      self.walls.append( wall(6*self.resolution,10*self.resolution+i,self,self.grid))
    for i in range(2*self.resolution):
      #self.grid[4*self.resolution+1+i,15*self.resolution] = case(self.def_zone(i,j)  , status = "wall" )
      self.walls.append( wall(4*self.resolution+1+i,15*self.resolution,self,self.grid) )

    for i in self.walls:
      self.grid = i.update(self,self.grid)

  def def_zone(self,i,j):
    zone = 1 
    if(i<6*self.resolution and j < 6*self.resolution):
      zone = 3
    if(i<6*self.resolution and j >= 10*self.resolution):
      zone = 4
    if(i>=12*self.resolution and j < 4*self.resolution):
      zone = 0
    if(i>=14*self.resolution and j < 2*self.resolution):
      zone = 0
    if(i>=12*self.resolution and j >= 8*self.resolution):
      zone = 2
    return zone



  def display(self):
    data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    for i in range(self.H):
      for j in range(self.W):
        data[i,j] = self.grid[i,j].color
    img = Image.fromarray(data, 'RGB')
    img.show()
  
  def update_robot(self,robot):
    self.grid = robot.display(self.grid,self)
    


    
          
        
  def update_map(self,robot,path,XY):

    self.build_map()
    for i in self.walls:
      self.grid = i.update(self,self.grid)
        #Make condition for explored area !
    self.update_robot(robot)
    self.image_path(path,XY,robot)
    
    
  def image_path(self,path,XY,robot):
    
    
    #self.grid[XY[0],XY[1]] = case(self.def_zone(XY[0],XY[1]),"checkpoint")
    for p in path:
      self.grid[p[0],p[1]] = case(self.def_zone(p[0],p[1]), "checkpoint",color=[0,255,0])

   
    for checkpoint in XY:
      for i in range(checkpoint[0]-round(robot.size_front/4),checkpoint[0]+round(robot.size_front/4)):
        for j in range(checkpoint[1]-round(robot.size_front/4),checkpoint[1]+round(robot.size_front/4)):
          
          self.grid[i,j] = case(self.def_zone(i,j), "checkpoint")
    
    



    
  
  def map_item(self):
    size = {"bottle": 2 , "obstacle" : 5}
    print(self.bottle)
    for bottle in self.bottle:
      for i in range(bottle[0] - size["bottle"] ,bottle[0] + size["bottle"] ):
        for j in range(bottle[1] - size["bottle"] ,bottle[1] + size["bottle"] ):

          self.grid[bottle[0],bottle[1]] = case(self.def_zone(bottle[0],bottle[1]) , status = "bottle" )

    for obstacle in self.obstacle:
      for i in range(obstacle[0] - size["obstacle"] ,obstacle[0] + size["obstacle"]  ):
        for j in range(obstacle[1] - size["obstacle"]  ,obstacle[1] + size["obstacle"]  ):
          self.grid[obstacle[0],obstacle[1]] = case(self.def_zone(obstacle[0],obstacle[1]) , status = "obstacle" )

  def new_item(self, type_, robot, sensor ): #sensor 0 to 13 
    
    x = robot.x
    y = robot.y
    a = robot.angle

    #1 pop object au dessus capteur
    #2 pop objet dans la bonne orientation du capteur, rotation de l'objet autour du capteur par l'angle
    #3 pop objet avec en plus prise en compte de l'angle du robot
    #Angle robot + angle sensor par rapport au robot (arctan(robot.ir_sensors[sensor][1]/robot.ir_sensors[sensor][0]))
    xi = robot.ir_sensors[sensor][0] + x-5
    yi = robot.ir_sensors[sensor][1] + y
    v1 = np.array([0,robot.size_front - y])
    v2 = np.array([xi -x ,  yi-y])
    v1_norm=  np.linalg.norm(v1,ord = 2)
    v2_norm = np.linalg.norm(v2,ord  = 2)
    
    angle_sensor = math.acos(np.dot(v1,v2)/(v1_norm*v2_norm))
    

    
    #xi,yi = self.rotation(xi,yi, robot.ir_sensors[sensor][0], robot.ir_sensors[sensor][1] ,a )

    if(type_ == "bottle"):
      self.bottle.append([xi,yi])
    else:
      self.obstacle.append([xi,yi])


    
  
        



  
  def map(self):
    return(self.grid)

  
  




