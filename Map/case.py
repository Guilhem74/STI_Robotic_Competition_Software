
import numpy as np
from PIL import Image
from heapq import *
import math
import threading
from collections import deque


class case:
  def __init__(self,zone,status,color = None):
    self.zone = zone
    
    self.status = status
    self.color = color
    self.update_col()
  def set_status(self,status):
    self.status = status
    self.update_col()

  def get_status(self):
    return self.status

  def update_col(self):
    if(self.color == None):
      if(self.status == "checkpoint"):
        self.color = [255,128,0]
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
          self.color = [255,0,0]
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
          self.color = [255,0,0]
        else:
          self.color = [0,0,255]
      if(self.status == "robot"):
        self.color =[255,255,0]