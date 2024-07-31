from enum import Enum
import math

class STATUS(Enum):
  UNKNOWN = 0
  NONEXISTENT = 1
  UNEXPLORED = 2
  DEADEND = 3
  CONNECTED = 4
  BLOCKED = 5

class Intersection():
  def __init__(self, x, y):
    self.x = x
    self.y = y
    self.dist = math.inf
    self.dir = None
    self.roads = [STATUS.UNKNOWN] * 8
    i = 0
    self.blocked = [False] * 8 
    
    
  def add_road(self, heading, status):
    self.roads[heading] = status

  def get_road(self, heading):
    return self.roads[heading]

  def get_unknown(self):
    unknown = []
    for (i,e) in enumerate(self.roads):
      if((e == STATUS.UNKNOWN or e == STATUS.UNEXPLORED) and not self.is_blocked(i)):
        unknown.append(i)
    return unknown
  
  def get_unexplored(self):
    for (i,e) in enumerate(self.roads):
      if(e == STATUS.UNEXPLORED):
        return i
    return None

  def get_roads(self):
    return self.roads
  
  def set_dist(self, dist):
    self.dist = dist
  
  def get_dist(self):
    return self.dist
  
  def set_dir(self, dir):
    self.dir = dir

  def get_dir(self):
    return self.dir
    
  def get_coords(self):
    return (self.x, self.y)
    
  def set_blocked(self, heading, blocked):
    self.blocked[heading] = blocked
  
  def is_blocked(self, heading):
    return self.blocked[heading]
  
  def clear_blocked(self):
    self.blocked = [False] * 8 
