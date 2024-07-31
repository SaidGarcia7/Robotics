from Intersection import *
import matplotlib.pyplot as plt
import math

ARROW = {0:(0, 1), 1:(-1, 1), 2:(-1, 0),
         3:(-1, -1), 4:(0, -1), 5:(1, -1),
         6:(1, 0), 7:(1, 1)}
         
LINES = {
    STATUS.UNKNOWN : "black",
    STATUS.NONEXISTENT : "lightgray",
    STATUS.UNEXPLORED : "blue",
    STATUS.DEADEND : "red",
    STATUS.CONNECTED : "green",
}
class Map():
  def __init__(self):
    self.intersections = {}

  def add_road(self, x, y, heading, status):
    if((x,y) in self.intersections):
      self.intersections[(x,y)].add_road(heading, status)
    
  def add_intersection(self, x, y):
    if((x,y) not in self.intersections):
      self.intersections[(x,y)] = Intersection(x,y)

  def update_intersection(self, x, y, dir, dist):
    if((x,y) in self.intersections):
      self.intersections[(x,y)].set_dir(dir)
      self.intersections[(x,y)].set_dist(dist)
  
  def clear(self):
    for (x,y) in self.intersections:
      i = self.intersections[(x,y)]
      i.set_dir(None)
      i.set_dist(math.inf)
  
  def get_intersection(self, x, y):
    if((x,y) in self.intersections):
      return self.intersections[(x,y)]
    else:
      print("Intersection not on map")
      return None
  
  def get_intersections(self):
    return self.intersections.values()
    
  def plot(self, heading, pos):
    # Clear the current, or create a new figure.
    plt.clf()
    # Create a new axes, enable the grid, and set axis limits.
    plt.axes()
    plt.gca().set_xlim(-5.5, 5.5)
    plt.gca().set_ylim(-5.5, 5.5)
    plt.gca().set_aspect('equal')
    # Show all the possible locations.
    for x in range(-5, 6):
      for y in range(-5, 6):
        plt.plot(x, y, color='lightgray', marker='o', markersize=8)
    # Plot bot heading.
    shift = ARROW[heading]
    xbase, ybase = pos
    xtip = xbase + (0.25 *shift[0])
    xbase = xbase - (0.25 *shift[0])
    ytip = ybase + (0.25 *shift[1])
    ybase = ybase - (0.25 *shift[1])
      
    plt.arrow(xbase, ybase, xtip-xbase, ytip-ybase,
              width=0.2,
              head_width=0.3,
              head_length=0.1,
              color='magenta')

    # Plot intersection lines.
    for (x,y) in self.intersections:
      i = self.intersections[(x,y)]
      if(i.get_dir() is not None):
        shift = ARROW[i.get_dir()]
        xbase = x + 0.25 * shift[0]
        xtip = x + 0.5 * shift[0]
        ybase = y + 0.25 * shift[1]
        ytip = y + 0.5 * shift[1]
        plt.arrow(xbase, ybase, xtip-xbase, ytip-ybase,
              width=0.1,
              head_width=0.2,
              head_length=0.1,
              color='cyan')
      for h in range(8):
        (xto, yto) = self.calc_line((x, y), h)
        if(i.is_blocked(h)):
          plt.plot([x, xto], [y, yto], color="orange")
        else:
          plt.plot([x, xto], [y, yto], color=LINES[self.intersections[(x,y)].roads[h]])
        
    # Show the graph and continue.
    #plt.pause(0.01)
    plt.savefig("plot.png")
    #eog plot.png
    

  def calc_line(self, tup, h):
    shift = ARROW[h]
    x, y = tup
    xto = x + (0.5 * shift[0])
    yto = y + (0.5 * shift[1])
    return (xto, yto)

  def get_connected_roads(self, intersection):
    if(intersection.get_coords() in self.intersections):
      roads = intersection.get_roads()
      connected_roads = []
      for i in range(8):
        if(roads[i] == STATUS.CONNECTED and not intersection.is_blocked(i)):
          (x,y) = intersection.get_coords()
          shift = ARROW[i]
          x = x + shift[0]
          y = y + shift[1]
          inter_connected = self.get_intersection(x,y)
          if(inter_connected is not None and not inter_connected.is_blocked((i-4)%8)):
            connected_roads.append(self.get_intersection(x,y))
      return connected_roads
    else:
      print("no roads")
      return None

  def update_blocked_status(self, x, y, heading, distance,d):
    
    if distance is not None and distance < d:
      self.intersections[(x, y)].set_blocked(heading, True)
    else:
      self.intersections[(x, y)].set_blocked(heading, False)

  def set_blocked(self, x, y, heading):
    if((x,y) in self.intersections):
      self.intersections[(x,y)].set_blocked(heading, True)
  
  def clear_blocked(self):
    for (x,y) in self.intersections:
      self.intersections[(x,y)].clear_blocked()
      
  def get_potential_goals(self, goal):
    pot_goals = {}
    for (x,y) in self.intersections:
      inter = self.get_intersection(x,y)
      if len(inter.get_unknown()) > 0:
        pot_goals[inter] = self.get_norm_dist(goal,(x,y))
    print(pot_goals)
    return pot_goals
    
  def get_norm_dist(self,goal, curr):
    return 2 * math.sqrt((goal[0]-curr[0]) ** 2 + (goal[1]-curr[1]) ** 2)
