import threading
class Shared():
  def __init__(self, goal, exploring, manual, map, pos, heading):
    self.robotx = 0
    self.roboty = 0
    self.robotheading = 0
    self.goal = goal
    self.exploring = exploring
    self.manual = manual
    self.map = map
    self.pos = pos
    self.heading = heading
    self.action = True
    self.direction = None
    self.lock = threading.Lock()
    self.map_shift = False
    self.map_clear = False
    self.directed_explore = False
    self.robotx = 0
    self.roboty = 0
    self.robotheading = 0

  def show(self):
    print(self.goal,self.exploring,self.manual,self.pos,
          self.heading,self.action,self.direction,self.map_shift)
  def set_goal(self, goal):
    self.goal = goal

  def set_exploring(self, exploring):
    self.exploring = exploring

  def set_map(self,map):
    self.map = map

  def set_manual(self, manual):
    self.manual = manual

  def get_map(self,map):
    return self.map
    
  def get_goal(self):
    return self.goal

  def get_exploring(self):
    return self.exploring

  def get_manual(self):
    return self.manual

  def acquire(self):
    print("acquired")
    return self.lock.acquire()
    

  def release(self):
    self.lock.release()
    print("released")
