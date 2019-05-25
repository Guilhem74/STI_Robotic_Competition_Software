#Map generator



import numpy as np
import math
import matplotlib.pyplot as plt




class Mapping:
  def __init__(self):

    self.resolution = 50
    self.bottle = []
    self.obstacle = []
    self.H = 16*self.resolution
    self.W = 16*self.resolution
    self.walls = []
    self.obstacle_size = 15
    self.bottle_size = 5
    #self.data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    self.grid = np.zeros((self.H,self.W), dtype=int)
    self.grid_reward = np.ones((self.H,self.W), dtype=int)
    #self.build_map(0)



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
    if G!= None:
        for coord in G:
          A[coord[0],coord[1],1] = G[coord[0],coord[1]]
    if path!= None:
        for coord in path:
          A[coord[1],coord[0],1] = 255
    Robot_Array=self.Get_Display_Pos_Robot(robot)

    A[:,:,2]=Robot_Array
    A[:,:,0]=A[:,:,0]/max(1,np.max(A[:,:,0]))
    A[:,:,1]=A[:,:,1]/max(1,np.max(A[:,:,1]))
    A[:,:,2]=A[:,:,2]/max(1,np.max(A[:,:,2]))
    plt.figure(figsize=(20,20))
    plt.imshow(A,origin='lower')
    plt.axis([max(0,robot.x-200),min(800,robot.x+200),max(0,robot.y-200),min(800,robot.y+200)])
    return A;
    
  def Get_Display_Pos_Robot(self,robot):
    A = np.zeros((self.H,self.W))
    cx = robot.x
    cy = robot.y
    corner = []
    #Front left corner of robot
    for i in range(robot.size_front):
        for j in range(robot.size_side):
            x0 = cx - i 
            y0 = cy - j
            corner.append([x0,y0])
            
    #Front right corner of robot
    for i in range(robot.size_front):
        for j in range(robot.size_side):
            x0 = cx - i 
            y0 = cy + j
            corner.append([x0,y0])
     
    #back left corner of robot
    for i in range(robot.size_back):
        for j in range(robot.size_side):
            x0 = cx + i 
            y0 = cy - j
            corner.append([x0,y0])
    
    #Back right corner of robot
    for i in range(robot.size_back):
        for j in range(robot.size_side):
            x0 = cx + i 
            y0 = cy + j
            corner.append([x0,y0])

    
        #Update corner robot
    for i in corner:
        x = i[0]
        y = i[1]
        x,y = self.rotation(x,y,cx,cy,-(robot.angle+math.pi/2))
        A[x,y] = 255
        A[cx,cy] = 255
    new_point = []
    #A[cy-20:cy+20,cx-20:cx+20]=100
    return A
    
    
  
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
            print("NEW OBSATCLE")
            self.obstacle.append(element)
        for obstacle in self.obstacle:
          for i in range(max(0,obstacle[0] - self.obstacle_size) ,min(self.H,obstacle[0] + self.obstacle_size )):
            for j in range(max(0,obstacle[1] - self.obstacle_size)  ,min(self.W,obstacle[1] + self.obstacle_size)  ):
              self.grid[j,i] = 255
        #self.grid = gaussian_filter(self.grid, sigma=7)
        return self.grid


    
  




  
  def map(self):
    return(self.grid)

  
  




