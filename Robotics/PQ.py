from Intersection import *

class priorityqueue():
  def __init__(self):
    self.queue = []


  def isEmpty(self):
    return len(self.queue) == 0

  def insert(self, node):
    if(self.isEmpty()):
        self.queue.append(node)
        return
    if(node.get_dist() > self.queue[len(self.queue)-1].get_dist()):
        self.queue.append(node)
        return
    for (i,e) in enumerate(self.queue):
      if (node.get_dist() <= e.get_dist()):
          self.queue.insert(i,node)
          return

  def pop(self):
    return self.queue.pop(0)


  def replace(self, node):
    for i in self.queue:
      if ((i.get_coords() == node.get_coords()) and (i.get_dist() > node.get_dist())):
          self.queue.remove(i)
          self.insert(node)
          break

  def contains(self, intersection):
    for i in self.queue:
      if(i.get_coords() == intersection.get_coords()):
        return True
    return False
