
from map import *
from robot import *
from pathfinder import *
from case import *

class sensor:
	def __init__(self,id_,x,y,a):
		self.x_ = x
		self.y_ = y
		self.a_ = a
		self.id = id_
	def set_position(self,robot):
		self.rx = robot.x
		self.ry = robot.y
		self.ra = robot.angle

		self.x = robot.x + self.x_
		self.y = robot.y + self.y_
		self.a = robot.angle + self.a_

		self.x,self.y = robot.rotation(self.x,self.y,robot.x,robot.y,robot.angle)

	def display(self,grid,map):
		
		grid[round(self.x),round(self.y)] = case(map.def_zone(round(self.x),round(self.y)) , status = "robot" , color=[255,0,0] )
		return grid



		



