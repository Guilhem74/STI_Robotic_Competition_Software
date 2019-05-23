#Map generator



import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter




class Mapping:
  def __init__(self):

    self.resolution = 50
    self.bottle = []
    self.obstacle = []
    self.H = 16*self.resolution
    self.W = 16*self.resolution
    self.walls = []
    self.obstacle_size = 10 + 13
    self.bottle_size = 5
    #self.data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    self.grid = np.ones((self.H,self.W), dtype=int)
    self.grid_reward = np.ones((self.H,self.W), dtype=int)
    self.build_map(0)



  def build_map(self,num):
    self.grid[[0,-1],:] = 255
    self.grid[:, [0,-1]] = 255
    zone = np.ones((6*self.resolution,6*self.resolution))*255
    if(num == 0):
        self.grid[800-6*self.resolution:800,:6*self.resolution] = zone
        self.grid[:6*self.resolution,800-6*self.resolution:800] = zone
        self.grid[800-6*self.resolution:800,800-6*self.resolution:800] = zone
    if(num == 1):
        self.grid[800-6*self.resolution:800,:6*self.resolution] = zone
        self.grid[800-6*self.resolution:800,800-6*self.resolution:800] = zone
    if(num == 2):
        self.grid[800-6*self.resolution:800,:6*self.resolution] = zone
        self.grid[800-6*self.resolution:800,799-6*self.resolution:801-6*self.resolution] = np.ones((6*self.resolution,2))*255
        self.grid[799-6*self.resolution:801-6*self.resolution,800-6*self.resolution:800-self.resolution] = np.ones((2,5*self.resolution))*255
        self.grid[800-6*self.resolution:800-4*self.resolution,799-self.resolution:801-self.resolution] = np.ones((2*self.resolution,2))*255
    

        
  



  def display(self,G,path,robot):
    A = np.zeros((self.H,self.W,3))    
    A[:,:,0] = self.grid[:,:]
      
    for coord in G:
      A[coord[0],coord[1],1] = G[coord[0],coord[1]]
    for coord in path:
      A[coord[1],coord[0],1] = 255
    Robot_Array=self.Get_Display_Pos_Robot(robot)

    A[:,:,2]=Robot_Array
    A[:,:,0]=A[:,:,0]/np.max(A[:,:,0])
    A[:,:,1]=A[:,:,1]/np.max(A[:,:,1])
    A[:,:,2]=A[:,:,2]/np.max(A[:,:,2])
    plt.figure(figsize=(20,20))
    plt.imshow(A/np.max(A),origin='lower')
    
    plt.axis([max(0,robot.x-200),min(800,robot.x+200),max(0,robot.y-200),min(800,robot.y+200)])
    return A
    
  def Get_Display_Pos_Robot(self,robot):
    A = np.zeros((self.H,self.W))
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
    x3 = cx + robot.size_back
    y3 = cy + robot.size_side
    corner = [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
    new_point = []
    A[cy-20:cy+20,cx-20:cx+20]=128
    return A
    
    
  

  

    
    
  def new_bottle(self, robot, sensor):
        x = robot.x
        y = robot.y
        a = robot.angle
        xi = robot.ir_sensors[sensor][0] + x
        yi = robot.ir_sensors[sensor][1] + y 
        self.bottle.append([xi,yi])

        for bottle in self.bottle:
            for i in range(bottle[0] - self.bottle_size ,bottle[0] + self.bottle_size ):
                for j in range(bottle[1] - self.bottle_size ,bottle[1] + self.bottle_size ):
                    self.grid_reward[bottle[0],bottle[1]] = -100
        #self.grid = gaussian_filter(self.grid, sigma=7)   



  def new_obstacle(self, xy ): 
        for element in xy:
            self.obstacle.append(element)
        print(self.obstacle)
        for obstacle in self.obstacle:
          for i in range(max(0,obstacle[0] - self.obstacle_size) ,min(self.H,obstacle[0] + self.obstacle_size )):
            for j in range(max(0,obstacle[1] - self.obstacle_size)  ,min(self.W,obstacle[1] + self.obstacle_size)  ):
              self.grid[j,i] = 255
        self.grid = gaussian_filter(self.grid, sigma=7)
        return self.grid


    
  
        



  
  def map(self):
    return(self.grid)

  
  




