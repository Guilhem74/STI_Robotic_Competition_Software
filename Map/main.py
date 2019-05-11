


    
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




if __name__ == '__main__':
  np.set_printoptions(threshold=np.inf)
  mapping = Map()
  pf = pathfinder()
  robot = robot(140,10,math.pi/4)
  path = pf.astar(mapping.grid,(30,145),(robot.x,robot.y))
  
  XY = pf.find_vertice(path)
  print(XY)
  if(path != False and XY != None):
    
    
    
    mapping.update_map(robot,path,XY)
    mapping.new_item('bottle',  robot, "B")
    mapping.map_item()
    mapping.display()


    