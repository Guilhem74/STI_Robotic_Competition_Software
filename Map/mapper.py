#Map generator

import numpy as np
from PIL import Image
from heapq import *
import math





class robot:
  def __init__(self,x,y,a):
    self.x = x
    self.y = y
    self.angle = a
    self.size = 6
    self.color = [255, 255, 0]
  def set_position(self, x,y,a):
    self.x = x
    self.y = y
    self.a = a
    

  def get_position(self):
    return [self.x, self.y, self.a]
    


class case:
  def __init__(self,zone,status):
    self.zone = zone
    self.status = status
    self.color = [0,0,0]
    self.update_col()
  def set_status(self,status):
    self.status = status
    self.update_col()

  def get_status(self):
    return self.status

  def update_col(self):
    if(self.status == "wall"):
      self.color = [255,0,0]
    if(self.status == "bottle"):
      self.color = [100,200,100]
    if(self.status == "obstacle"):
      self.color = [255,0,0]
    if(self.status == "unexplored"):
      if(self.zone == 0):
        self.color = [0,100,100]
      elif(self.zone == 1):
        self.color = [125,125,125]
      elif(self.zone == 2):
        self.color = [100,0,100]
      elif(self.zone == 3):
        self.color = [0,100,0]
      else:
        self.color = [0,0,100]
    if(self.status == "explored"):
      if(self.zone == 0):
        self.color = [0,255,255]
      elif(self.zone == 1):
        self.color = [255,255,0]
      elif(self.zone == 2):
        self.color = [255,0,255]
      elif(self.zone == 3):
        self.color = [0,255,00]
      else:
        self.color = [0,0,255]




class Map:
  def __init__(self):

    self.resolution = 10
    self.size_robot = 1 * self.resolution
    self.H = 16*self.resolution
    self.W = 16*self.resolution
    self.data = np.zeros((self.H, self.W,3), dtype=np.uint8)
    self.map = np.zeros((self.H,self.W), dtype=int)
    self.grid = np.empty((self.H+20,self.W+20), dtype=object)
    self.build_map()



  def build_map(self):
    self.grid[:] = case(zone = 1 , status = "unexplored" )
    for i in range(self.H):
      for j in range(self.W):
        if(i<6*self.resolution and j < 6*self.resolution):
          self.map[i,j] = 0
          self.grid[i,j] = case(zone = 3 , status = "wall" )
        if(i<6*self.resolution and j >= 10*self.resolution):
          self.map[i,j] = 4
          self.grid[i,j] = case(zone = 4 , status = "unexplored" )
        if(i>=12*self.resolution and j < 4*self.resolution):
          self.map[i,j] = 0
          self.grid[i,j] = case(zone = 0 , status = "unexplored" )
        if(i>=14*self.resolution and j < 2*self.resolution):
          self.map[i,j] = 5
          self.grid[i,j] = case(zone = 0 , status = "explored" )
        if(i>=12*self.resolution and j >= 8*self.resolution):
          self.grid[i,j] = case(zone = 2 , status = "unexplored" )
          self.map[i,j] = 2
        if(i==0 or i==self.H-1 or j ==0 or j ==self.W-1):
          self.grid[i,j] = case(zone = 2 , status = "wall" )

    for i in range(6*self.resolution):
      self.grid[i,10*self.resolution] = case(zone = 2 , status = "wall" )
    for i in range(5*self.resolution):
      self.grid[6*self.resolution,10*self.resolution+i] = case(zone = 2 , status = "wall" )
    for i in range(2*self.resolution):
      self.grid[4*self.resolution+1+i,15*self.resolution] = case(zone = 2 , status = "wall" )
      







  def display(self):
    img = Image.fromarray(self.data, 'RGB')
    img.show()
    
  def update_image(self,robot):

    for i in range(self.H):
      for j in range(self.W):

        self.data[i,j] = self.grid[i,j].color
    for i in range(round(robot.x - robot.size/2 ), round(robot.x + robot.size/2)):
      for j in range(round(robot.y - robot.size/2 ), round(robot.y + robot.size/2)):
        self.data[i,j] = robot.color

    
  def image_path(self,path,XY):
    
    self.data[XY[0],XY[1]] = [128,50,255]

    for p in path:

      self.data[p[0],p[1]] = [0,255,0]
    for i in range(self.H):
      for j in range(self.W):
        if((i - XY[0])*(i - XY[0])   +(j - XY[1])*(j - XY[1]) < 5):
          self.data[i,j] = [178,180,50]
    
    



    
  


  def new_item(self, type_, position_robot, distance, sensor ): #sensor 0 to 13 
    sensors_angle={0:0, 1:40, 2:60, 3:80 , 4:100, 5:140, 6:180, 7:220, 8:240, 9:260, 10:280, 11:320, 12:340}
    sensors_angle[sensor]

    size = {"bottle": 2 , "obstacle" : 5}
    for i in range(round(position_robot[0] + math.cos(sensors_angle[position_robot[2]])*distance -size[type_]) , round(position_robot[0] + math.cos(sensors_angle[position_robot[2]])*distance + size[type_])) :
      for j in range(round(position_robot[1] + math.sin(sensors_angle[position_robot[2]])*distance -size[type_]) , round(position_robot[1] + math.sin(sensors_angle[position_robot[2]])*distance + size[type_])):
        self.grid[i,j] = case(zone = 0 , status = type_ )
        self.data[i,j] = self.grid[i,j].color
    



  
  def map(self):
    return(self.map)

  def heuristic(self, a, b):
      return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

  def astar(self, array,  goal,start):

      neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

      close_set = set()
      came_from = {}
      gscore = {start:0}
      fscore = {start:self.heuristic(start, goal)}
      oheap = []

      heappush(oheap, (fscore[start], start))
      
      while oheap:

          current = heappop(oheap)[1]

          if current == goal:
              data = []
              while current in came_from:
                  data.append(current)
                  current = came_from[current]
              return data

          close_set.add(current)
          for i, j in neighbors:
              neighbor = current[0] + i, current[1] + j            
              tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
              if 0 <= neighbor[0] < array.shape[0]:
                  if 0 <= neighbor[1] < array.shape[1]:                
                      if array[neighbor[0]][neighbor[1]].status == "wall" or array[neighbor[0]][neighbor[1]].status == "obstacle":
                          continue
                  else:
                      # array bound y walls
                      continue
              else:
                  # array bound x walls
                  continue
                  
              if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                  continue
                  
              if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                  came_from[neighbor] = current
                  gscore[neighbor] = tentative_g_score
                  fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                  heappush(oheap, (fscore[neighbor], neighbor))
                  
      return False


def find_vertice(path):


  
  #####################################"      MISE EN FORME DE LA TRAJEC POUR DES LIGNES DROITES
  listedecime = []
  listeangle = []
  virage = []
  dd = []
  #for i in range(0, len(PATH), 10):
  # listedecime.append(PATH[i])
   
  listedecime = path
  
  for i in range(len(listedecime)-2):
    ax = listedecime[i][0]
    ay = listedecime[i][1]
    bx = listedecime[i+1][0]
    by = listedecime[i+1][1]
    cx = listedecime[i+2][0]
    cy = listedecime[i+2][1]
    AB = np.sqrt((ax - bx)*(ax - bx)+ (ay - by)*(ay - by))
    BC = np.sqrt((bx - cx)*(bx - cx) + (by - cy)*(by - cy))
    CA = np.sqrt((cx - ax)*(cx - ax) + (cy - ay)*(cy - ay))
    angle = math.acos((AB*AB + BC*BC - CA*CA) / (2 * AB * BC))
    angle = angle/np.pi * 360
    listeangle.append(angle)
    
    if (angle < 330):
      virage.append(1)
    else:
      virage.append(0)
    
  n = 0
  for i in range(len(virage)):
      if(virage[i] == 1):
          n=n+1
      else:
          if(n>1):
              dd.append(int(i-round(n/2)))
              n=0
          elif(n==1):
              dd.append(i)
              n=0
          else:
              n=0
  
  for i in dd:
    return path[i]
  


if __name__ == '__main__':
  np.set_printoptions(threshold=np.inf)
  mapping = Map()
  robot = robot(150,10,0)
  path = mapping.astar(mapping.grid,(50,157),(159,0))
  print(path)
  XY = find_vertice(path)
  print(XY)
  if(path != False):

    
    mapping.update_image(robot)
    mapping.image_path(path,XY)
    mapping.new_item('bottle',  [robot.x,robot.y,robot.angle], 5, 5)
    mapping.display()
    

