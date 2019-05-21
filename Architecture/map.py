#Map generator



import numpy as np
import math
import matplotlib.pyplot as plt



class Map:
  def __init__(self):

    self.resolution = 50
    self.bottle = []
    self.obstacle = []
    self.H = 16*self.resolution
    self.W = 16*self.resolution
    self.walls = []
    #self.data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    self.grid = np.ones((self.H,self.W), dtype=int)
    self.grid_reward = np.ones((self.H,self.W), dtype=int)
    
    
    

    #self.build_map()



  def build_map(self):
    
    
    for i in range(self.H):
      for j in range(self.W):
        self.grid[i,j] = 99

        
        
        if(i==0 or i==self.H-1 or j ==0 or j ==self.W-1):
          
          self.grid[i,j] = 99


    for i in range(6*self.resolution):
      self.grid[i,10*self.resolution] =  99
      

    for i in range(5*self.resolution):
      self.grid[6*self.resolution,10*self.resolution+i] = 99
      
    for i in range(2*self.resolution):
      self.grid[4*self.resolution+1+i,15*self.resolution] = 99
      



  



  def display(self,G,path,robot):
    A = np.zeros((self.H,self.W,3))    
    A[:,:,0] = self.grid[:,:]
    #A = cv2.GaussianBlur(A/np.max(A),(5,5),0)   
    for coord in G:
      A[coord[0],coord[1],1] = G[coord[0],coord[1]]
    for coord in path:
      A[coord[1],coord[0],1] = 255
    Robot_Array=self.Get_Display_Pos_Robot(robot)
    # A = robot.display(A)
    A[:,:,2]=Robot_Array
    
    
    plt.figure(figsize=(20,20))
    plt.imshow(A/np.max(A),origin='lower')
    plt.axis([0,500,0,500])
    
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
    
    
  

  
  def map_item(self):
    size = {"bottle": 1 , "obstacle" : 10+13}
    
    for bottle in self.bottle:
      for i in range(bottle[0] - size["bottle"] ,bottle[0] + size["bottle"] ):
        for j in range(bottle[1] - size["bottle"] ,bottle[1] + size["bottle"] ):

          self.grid_reward[bottle[0],bottle[1]] = -100
    
    for obstacle in self.obstacle:
      for i in range(obstacle[0] - size["obstacle"] ,obstacle[0] + size["obstacle"]  ):
        for j in range(obstacle[1] - size["obstacle"]  ,obstacle[1] + size["obstacle"]  ):
          self.grid[j,i] = 255
    
    
    



  def new_item(self, type_, robot, sensor ): #sensor 0 to 13 
    
    x = robot.x
    y = robot.y
    a = robot.angle

    
    xi = robot.ir_sensors[sensor][0] + x
    yi = robot.ir_sensors[sensor][1] + y
   
    
    if(type_ == "bottle"):
      self.bottle.append([xi,yi])
    else:
     
      self.obstacle.append([xi,yi])
    self.map_item()



    
  
        



  
  def map(self):
    return(self.grid)

  
  




