
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



class wall:
	def __init__(self,x,y,map,grid):
		self.size = 4
		self.x = x
		self.y = y
		
	def update(self,map,grid):
		for i in range(self.x - self.size , self.x + self.size  ):
			for j in range(self.y - self.size , self.y + self.size  ):
				grid[i,j] = case(map.def_zone(i,j) , status = "wall" )
		return grid

class obstacle:
	def __init__(self,x,y):
		self.size = 4
		self.x = x
		self.y = y

	def update(self,grid,map):
		for i in range(self.x - self.size , self.x + self.size  ):
			for j in range(self.y - self.size , self.y + self.size  ):
				grid[i,j] = case(map.def_zone(i,j) , status = "obstacle"  )
		return grid

