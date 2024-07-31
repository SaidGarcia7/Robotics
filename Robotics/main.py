from IR import *
from IR_man import *
from motor import *
from drive import *
from AngleSensor import *
from Map import *
from Intersection import *
import math
from PQ import *
from Ultrasound_man import *

import pickle
import pigpio
import sys
import time
import traceback
INTERSECTION = ["right_spin", "left_spin", "stop", "straight"]

HEADING = [0,1,2,3,4,5,6,7]

REL_HEADING = {(0, 1):0, (-1, 1):1, (-1, 0):2,
 (-1, -1):3, (0, -1):4, (1, -1):5,
 (1, 0):6, (1, 1):7}

DICT_SO = {(0,1,0) : "stop", (0,0,1) : "right_spin", (0,1,1) : "right_spin", (1,0,0) : "left_spin", (1,1,0) : "left_spin", (1,1,1) : "straight"}

DICT_LINE = {(1,0,1): "straight", (0,0,0): "straight",(0,1,0) : "straight", (0,0,1) : "right_hook", (0,1,1) : "right_turn", (1,0,0) : "left_hook", (1,1,0) : "left_turn", (1,1,1) : "straight"}

DICT_INTERSECTION = {(0,0,0): 0,(0,1,0) : 0, (0,0,1) : 0, (0,1,1) : 0, (1,0,0) : 0, (1,1,0) : 0, (1,1,1) : 1}

DICT_TURN = {(0,0,0): 0,(0,1,0) : 1, (0,0,1) : 1, (0,1,1) : 1, (1,0,0) : 1, (1,1,0) : 1}

DICT_CENT = {(0,0,0): 0,(0,1,0) : 1, (0,0,1) : 0, (0,1,1) : 0, (1,0,0) : 0, (1,1,0) : 0}

DICT_PUSH = {(0,1,0) : 0, (0,0,1) : -1, (0,1,1) : -0.5, (1,0,0) : 1, (1,1,0) : 0.5}

DICT_END = {(0,0,0): 1,(0,1,0) : 0, (0,0,1) : 0, (0,1,1) : 0, (1,0,0) : 0, (1,1,0) : 0, (1,1,1) : 0}

HEADING_DISTANCE = {0: 1, 1: 1.4, 2: 1, 3: 1.4, 4: 1, 5: 1.4, 6: 1, 7: 1.4}

def check_ahead(ultrasound_sensor, drive_man, heading, pos, map, dist, t):
  blocked = False
  drive_man.stop()
  t_start = time.time()
  while time.time() -t_start < t:
    distances = ultrasound_sensor.read()
    map.update_blocked_status(pos[0], pos[1],heading,distances,dist)
    if(distances < dist):
      blocked = True
  return blocked


def find_goal(map):
  lst = map.get_intersections()
  for i in lst:
    if(i.get_unknown() != []):
      return i

def calc_turn(d,h, map, pos):
  heading = h
  if(d == h):
    return "straight"
  else:
    count = 0
    while(d != h):
      h = (h + 1) % 8
      count += 1
    if(count <= 3):
      return "left_spin"
    elif(count == 4):
      inter = map.get_intersection(pos[0], pos[1])
      while((d-1)%8 != heading):
        heading = (heading + 1) % 8
        if(inter.get_road(heading) != STATUS.NONEXISTENT or inter.get_road(heading) != STATUS.UNKNOWN):
          return "right_spin"
      return "left_spin"
    else:
      return "right_spin"

def turning(direction, drive_man, ir_man):
  t_start = time.time()
  off_road = False
  drive_man.stop()
  turn_lvl = 0
  off_lvl = 1.0
  t_prev = time.time()
  while True:
    if (turn_lvl >= 0.8 and off_road):
      break
    curr_time = time.time()
    dt = curr_time - t_prev
    t_prev = curr_time
    tup = ir_man.read()
    drive_man.drive(direction)
    
    if (tup in DICT_TURN and off_road):
      turn_lvl = turn_lvl + (dt / 0.07) * (DICT_TURN[tup] - turn_lvl)
    (x, y, z) = tup
    off_lvl = off_lvl + (dt/0.15) * (y - off_lvl)
    if(off_lvl < 0.15):
      off_road = True
      
  t_end = time.time()
  drive_man.stop()
  return t_end - t_start

def straighten_out(drive_man, ir_man):
  print("straightening")
  drive_man.stop()
  tup = ir_man.read()
  centered_lvl = 0.5
  dt = 0 
  prev_time = time.time()
  while True:
    tup = ir_man.read()
    curr_time = time.time()
    dt = curr_time - prev_time
    prev_time = curr_time
    if(tup in DICT_SO):
      drive_man.drive(DICT_SO[tup])
    if (tup in DICT_TURN):
      centered_lvl = centered_lvl + (dt / 0.05) * (DICT_CENT[tup] - centered_lvl)
    if(centered_lvl > 0.8):
      break
  drive_man.stop()
    
def pull_forward(drive_man, map, heading, pos, ir_man):
  t_start = time.time()
  drive_man.stop()
  pull_lvl = 0.5
  t_prev = time.time()
  
  while (time.time() - t_start < 0.37):
    drive_man.drive("straight")
  drive_man.stop()

  t_start = time.time()
  while(time.time() - t_start < 0.3):
    curr_time = time.time()
    dt = curr_time - t_prev
    t_prev = curr_time
    (x,y,z) = ir_man.read()
    pull_lvl = pull_lvl + (dt / 0.05) * (y - pull_lvl)

  if(pull_lvl >= 0.8):
    if(map.get_intersection(pos[0], pos[1]).get_road(heading) == STATUS.UNKNOWN):
      map.add_road(pos[0], pos[1], heading, STATUS.UNEXPLORED)
      map.add_road(pos[0], pos[1], (heading-1)%8, STATUS.NONEXISTENT)
      map.add_road(pos[0], pos[1], (heading+1)%8, STATUS.NONEXISTENT)
  else:
    map.add_road(pos[0], pos[1], heading, STATUS.NONEXISTENT)

def line_following(ultrasound_sensor, drive_man, ir_man, inter_lvl, heading, pos, map):
  push_lvl = 0
  push_dir = 0
  inter_lvl = 0.5
  end_lvl = 0.5
  dt = 0
  t_prev = time.time()
  dead_end = False
  blocked = False
  while True:
    #push/end determination
    if(end_lvl >= 0.8 and push_dir == 1):
      drive_man.drive("left_hook")
    elif(end_lvl >= 0.8 and push_dir == -1):
      drive_man.drive("right_hook")
    elif(end_lvl >= 0.8):
      t_start = time.time()
      drive_man.stop()
      t_prev = time.time()
  
      while (time.time() - t_start < 0.4):
        drive_man.drive("straight")
      drive_man.stop()
      
      while True:
        if(end_lvl <= 0.2):
          break
        drive_man.drive("right_spin")
        tup = ir_man.read()
        if (tup in DICT_END):
          end_lvl = end_lvl + (dt / 0.05) * (DICT_END[tup] - end_lvl)
      
      map.add_road(pos[0], pos[1], heading, STATUS.DEADEND)
      pos = calc_move(pos, heading)
      heading = (heading - 4) % 8
      dead_end = True
      print("deadend")

    middle_ultra = ultrasound_sensor.read()
    if (middle_ultra != None and middle_ultra < 0.15):
      drive_man.drive("stop")
      blocked = True
      print("blocked")
      break
  
    #intersection determination
    if(inter_lvl >= 0.8):
      break

    #line following
    tup = ir_man.read()
    drive_man.drive(DICT_LINE[tup])
    curr_time = time.time()
    dt = curr_time - t_prev
    t_prev = curr_time

    #lvl update
    if (tup in DICT_INTERSECTION):
      inter_lvl = inter_lvl + (dt / 0.06) * (DICT_INTERSECTION[tup] - inter_lvl)
    if (tup in DICT_PUSH):
      push_lvl = push_lvl + (dt / 0.01) * (DICT_PUSH[tup] - push_lvl)
    if (tup in DICT_END):
      end_lvl = end_lvl + (dt / 0.05) * (DICT_END[tup] - end_lvl)

    #update push_dir
    if(push_lvl >= 0.8):
      push_dir = 1
    elif(push_lvl <= -0.8):
      push_dir = -1
    elif(-0.2 >= push_lvl <= 0.2):
      push_dir = 0
  
  drive_man.stop()
  return (inter_lvl,heading, dead_end, blocked)

def calc_move(old_pos, heading):
  if(heading == 0):
    return (old_pos[0], old_pos[1]+1)
  elif(heading == 1):
    return (old_pos[0]-1, old_pos[1]+1)
  elif(heading == 2):
    return (old_pos[0]-1, old_pos[1])
  elif(heading == 3):
    return (old_pos[0]-1, old_pos[1]-1)
  elif(heading == 4):
    return (old_pos[0], old_pos[1]-1)
  elif(heading == 5):
    return (old_pos[0]+1, old_pos[1]-1)
  elif(heading == 6):
    return (old_pos[0]+1, old_pos[1])
  elif(heading == 7):
    return (old_pos[0]+1, old_pos[1]+1)
  else:
    return old_pos

def calc_heading(old_heading, turn, dir):
  dif = 0
  print("Turn angle:" + str(turn))
  if(turn < 20):
    dif = 0
  elif(turn < 60):
    dif = 1
  elif(turn < 115):
    dif = 2
  elif(turn < 155):
    dif = 3
  elif(turn < 210):
    dif = 4
  elif(turn < 250):
    dif = 5
  elif(turn < 300):
    dif = 6
  else:
    dif = 0
  
  if(dir == "right_spin"):
    return (old_heading - dif) % 8
  else:
    return (old_heading + dif) % 8
    
def true_angle(a, a_init, direction):
  if(direction == "right_spin"):
    if((a_init == abs(a_init)) and (a != abs(a))):
      return (180 - a_init) + (180 + a)
    else:
      if(a_init > a):
        return 360 - abs(a - a_init)
      else:
        return abs(a - a_init)
  
  if(direction == "left_spin"):
    if((a_init != abs(a_init)) and (a == abs(a))):
      return (180 + a_init) + (180 - a)
    else:
      if(a_init < a):
        return 360 - abs(a - a_init)
      else:
        return abs(a - a_init)

def DNE_roads(map, old_heading, new_heading, direction, pos):
  if(direction == "right_spin"):
    old_heading = (old_heading - 1) % 8
    while(old_heading != new_heading):
      map.add_road(pos[0], pos[1],old_heading , STATUS.NONEXISTENT)
      old_heading = (old_heading - 1) % 8
  if(direction == "left_spin"):
    old_heading = (old_heading + 1) % 8
    while(old_heading != new_heading):
      map.add_road(pos[0], pos[1],old_heading , STATUS.NONEXISTENT)
      old_heading = (old_heading + 1) % 8

def djkstra(map, goal):
    if(map.get_connected_roads(goal) is None):
      print("Intersection not on current map")
      return False
    map.clear()
    on_deck = priorityqueue()
    done = []
    goal.set_dist(0)
    on_deck.insert(goal)
    while (not on_deck.isEmpty()):
      curr = on_deck.pop()
      n = map.get_connected_roads(curr)
      done.append(curr)
      for i in n:
        if(i not in done):
          old = curr.get_coords()
          new = i.get_coords()
          h = (new[0] - old[0], new[1] - old[1])
          temp_dist = curr.get_dist() + HEADING_DISTANCE[REL_HEADING[h]]
          if((on_deck.contains(i)) and (temp_dist < i.get_dist())):
            i.set_dist(temp_dist)
            (x,y) = i.get_coords()
            h = (old[0] - new[0], old[1] - new[1])
            i.set_dir(REL_HEADING[h])
            map.update_intersection(x,y,REL_HEADING[h], temp_dist)
            temp = Intersection(x,y)
            temp.set_dist(temp_dist)
            on_deck.replace(i)
            
          elif(not on_deck.contains(i)):
            i.set_dist(temp_dist)
            (x,y) = i.get_coords()
            h = (old[0] - new[0], old[1] - new[1])
            i.set_dir(REL_HEADING[h])
            map.update_intersection(x,y,REL_HEADING[h], temp_dist)
            on_deck.insert(i)
    return True

def decide_turn(goal, curr, heading, map,pos):
  desired = []
  x = goal[0]-curr[0]
  y = goal[1]-curr[1]
  if(x == 0):
    pass
  elif(x != abs(x)):
    desired.append(1)
    desired.append(2)
    desired.append(3)
  else:
    desired.append(7)
    desired.append(6)
    desired.append(5)
  if(y == 0):
    pass
  elif(y == abs(y)):
    desired.append(1)
    desired.append(0)
    desired.append(7)
  else:
    desired.append(5)
    desired.append(4)
    desired.append(3)
  unknown = map.get_intersection(curr[0], curr[1]).get_unknown()
  if len(unknown) == 0:
    return None
  else:
    print(desired)
    for i in unknown:
      if(i in desired):
        c = calc_turn(i,heading,map,pos)
        print(c)
        return c
    
    desired = unknown.pop(0)
    direction = calc_turn(desired, heading, map, pos)
    print(direction)
    return direction

def dir_djkstra(map, goals):
      map.clear()
      on_deck = priorityqueue()
      for i in goals:
        i.set_dist(goals[i])
        on_deck.insert(i)
      done = []
      while (not on_deck.isEmpty()):
        curr = on_deck.pop()
        n = map.get_connected_roads(curr)
        done.append(curr)
        for i in n:
          if(i not in done):
            old = curr.get_coords()
            new = i.get_coords()
            h = (new[0] - old[0], new[1] - old[1])
            temp_dist = curr.get_dist() + HEADING_DISTANCE[REL_HEADING[h]]
            if((on_deck.contains(i)) and (temp_dist < i.get_dist())):
              i.set_dist(temp_dist)
              (x,y) = i.get_coords()
              h = (old[0] - new[0], old[1] - new[1])
              i.set_dir(REL_HEADING[h])
              map.update_intersection(x,y,REL_HEADING[h], temp_dist)
              temp = Intersection(x,y)
              temp.set_dist(temp_dist)
              on_deck.replace(i)

            elif(not on_deck.contains(i)):
              i.set_dist(temp_dist)
              (x,y) = i.get_coords()
              h = (old[0] - new[0], old[1] - new[1])
              i.set_dir(REL_HEADING[h])
              map.update_intersection(x,y,REL_HEADING[h], temp_dist)
              on_deck.insert(i)
      return True


if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  ir_man = IR_man(io, 14, 15, 18)
  drive_man = Drive_Manager(io, 7, 8, 5, 6)
  t_prev = time.time()
  inter_lvl = 0.5 
  turns = []
  heading = 0
  dead_end = False
  blocked = False
  pos = (0,0)
  angle_sensor = AngleSensor(io, 27, 4, 17, [9,10,11,12,22,23,24,25])
  ultrasound_sensor = Ultrasound_man(io, 13, 19,26,16,20,21)
  map = Map()
  print("Would you like to load a map?")
  goal = None
  while True:
    ans = input()
    if(ans == "n"):
      break
    elif(ans == "y"):
        with open("1.pickle", "rb") as f:
          map = pickle.load(f)
          print("What is the goal x:y?(i.e 1:0)")
          goal_input = None
          while (goal_input is None):
            goal_input = input()
          g_list = goal_input.split(":")
          x = int(g_list[0])
          y = int(g_list[1])
          tup = (x,y)
          map.plot(heading, pos)
          goal = map.get_intersection(x,y)
          djkstra(map, goal)
          print("What is the start x:y:h?(i.e 1:0:0)")
          s_input = None
          while (s_input is None):
            s_input = input()
          s_list = s_input.split(":")
          x = int(s_list[0])
          y = int(s_list[1])
          heading = int(s_list[2])
          pos = (x, y)
        break
  try:
    inter_lvl, heading, dead_end,blocked = line_following(ultrasound_sensor, drive_man, ir_man, inter_lvl, heading, pos,map)
    map.add_intersection(pos[0], pos[1])
    pull_forward(drive_man, map, heading, pos, ir_man)
    manual = False
    exploring = False
    while True:
      if(goal is None):
        map.plot(heading, pos)
        print("Would you like to explore(e), goal find (g), manual(m)?")
        while True:
          ans = input()
          if(ans == "e"):
            goal = find_goal(map)
            if(goal is None):
              print("Fully explored")
            else:
              djkstra(map, goal)
              manual = False
              exploring = True
              break
          elif(ans == "g"):
            print("What is the goal x:y?(i.e 1:0)")
            goal_input = None
            while (goal_input is None):
              goal_input = input()
            g_list = goal_input.split(":")
            x = int(g_list[0])
            y = int(g_list[1])
            tup = (x,y)
            map.plot(heading, pos)
            goal = map.get_intersection(x,y)
            djkstra(map, goal)
            manual = False
            exploring = False
            break
          elif(ans == "m"):
            manual = True
            exploring = False
            break

      if(inter_lvl >= 0.8):

        #autonomous driving
        desired = None
        at_goal = False
        curr = map.get_intersection(pos[0], pos[1])
        if(exploring):
          if(curr is not None and len(curr.get_unknown()) > 0):
            goal = curr
            djkstra(map, goal)              
          else:
            goal = find_goal(map)
            path = djkstra(map, goal)
            if(path and curr.get_dir() is None):
              print("All paths blocked")
              map.clear_blocked()
              djkstra(map, goal)
            
        
        if(goal is not None and not manual):
          if(curr.get_dist() == 0):
            print("Reached Goal")
            at_goal = True
            goal = None
            if(not exploring):
              print("Would you like to go to a different goal?")
              while True:
                answer = input()
                if answer == "n":
                  curr = map.get_intersection(pos[0],pos[1])
                  n = map.get_connected_roads(curr)
                  old = curr.get_coords()
                  new = n[0].get_coords()
                  h = (new[0] - old[0], new[1] - old[1])
                  desired = REL_HEADING[h]
                  direction = calc_turn(desired, heading)
                  break
                if answer == "y":
                  print("What is the goal x:y?(i.e 1:0)")
                  goal_input = None
                  while (goal_input is None):
                    goal_input = input()
                  g_list = goal_input.split(":")
                  x = int(g_list[0])
                  y = int(g_list[1])
                  tup = (x,y)
                  map.plot(heading, pos)
                  goal = map.get_intersection(x,y)
                  djkstra(map, goal)
                  manual = False
                  exploring = False
                  break


          elif curr is not None:
            desired = curr.get_dir()
            if(desired is not None):
              direction = calc_turn(desired, heading)
            else:
              if(exploring):
                at_goal = True
              else:
                print("Input")
                while True:
                    direction = input()
                    if(direction in INTERSECTION):
                      break
          else:
            print("Error: Current position does not have an intersection.")
            break
        #manual driving
        else:  
          print("Input Direction:")
          while True:
              direction = input()
              if(direction in INTERSECTION):
                break

        #execution
        if(at_goal and curr is not None and exploring):
          unknown = curr.get_unknown()
          if(len(unknown) > 0):
            desired = unknown.pop(0)
            direction = calc_turn(desired, heading)
            if(direction != "straight"):
              a_init = angle_sensor.read()
              turning(direction, drive_man, ir_man)
              straighten_out(drive_man, ir_man)
              drive_man.drive("stop")
              a = angle_sensor.read()
              true_a = true_angle(a, a_init, direction)
              old_heading = heading
              heading = calc_heading(heading, true_a, direction)
              DNE_roads(map, old_heading, heading, direction, pos)
              curr = map.get_intersection(pos[0], pos[1])
              t_start = time.time()
              pull_lvl = 0.5
              while(time.time() - t_start < 0.3):
                curr_time = time.time()
                dt = curr_time - t_prev
                t_prev = curr_time
                (x,y,z) = ir_man.read()
                pull_lvl = pull_lvl + (dt / 0.05) * (y - pull_lvl)

              if(pull_lvl >= 0.8 and curr is not None):
                if(curr.get_road(heading) == STATUS.UNKNOWN):
                  map.add_road(pos[0], pos[1], (heading-1) % 8, STATUS.NONEXISTENT)
                  map.add_road(pos[0], pos[1], (heading+1) % 8, STATUS.NONEXISTENT)
                  map.add_road(pos[0], pos[1], heading, STATUS.UNEXPLORED)
              else:
                map.add_road(pos[0], pos[1], heading, STATUS.NONEXISTENT)
            map.plot(heading, pos)
          direction = "straight"
          if(curr is not None and curr.get_unexplored() is not None):
            print("unexplored")
            goal = curr
          
          elif(curr is not None and len(curr.get_unknown()) > 0):
            goal = curr
            print("c")
            n = map.get_connected_roads(curr)
            if(len(n) > 0):
              old = curr.get_coords()
              new = n[0].get_coords()
              h = (new[0] - old[0], new[1] - old[1])
              direction = "straight"
            else:
              djkstra(map, goal)
              unknown = curr.get_unknown()
              direction = "straight"
          else:
            goal = find_goal(map)
            if(goal is None):
              print("Fully explored")
            else:  
              print("new")
              djkstra(map, goal)
            curr = map.get_intersection(pos[0],pos[1])
            n = map.get_connected_roads(curr)
            old = curr.get_coords()
            new = n[0].get_coords()
            h = (new[0] - old[0], new[1] - old[1])
            direction = "straight"


        if(direction == "right_spin" or direction == "left_spin"):
          if(desired is not None):
            if(heading != desired):
              a_init = angle_sensor.read()
              turns.append(turning(direction, drive_man, ir_man))
              straighten_out(drive_man, ir_man)
              a = angle_sensor.read()
              true_a = true_angle(a, a_init, direction)
              old_heading = heading
              heading = calc_heading(heading, true_a, direction)
              DNE_roads(map, old_heading, heading, direction, pos)
              curr = map.get_intersection(pos[0], pos[1])
              drive_man.stop()
              t_start = time.time()
              pull_lvl = 0.5
              while(time.time() - t_start < 0.3):
                curr_time = time.time()
                dt = curr_time - t_prev
                t_prev = curr_time
                (x,y,z) = ir_man.read()
                pull_lvl = pull_lvl + (dt / 0.05) * (y - pull_lvl)

              if(pull_lvl >= 0.8 and curr is not None):
                if(curr.get_road(heading) == STATUS.UNKNOWN):
                  map.add_road(pos[0], pos[1], (heading-1) % 8, STATUS.NONEXISTENT)
                  map.add_road(pos[0], pos[1], (heading+1) % 8, STATUS.NONEXISTENT)
                  map.add_road(pos[0], pos[1], heading, STATUS.UNEXPLORED)
              else:
                map.add_road(pos[0], pos[1], heading, STATUS.NONEXISTENT)
              map.plot(heading, pos)
          else:
            a_init = angle_sensor.read()
            turns.append(turning(direction, drive_man, ir_man))
            straighten_out(drive_man, ir_man)
            a = angle_sensor.read()
            true_a = true_angle(a, a_init, direction)
            old_heading = heading
            heading = calc_heading(heading, true_a, direction)
            DNE_roads(map, old_heading, heading, direction, pos)

        elif(direction == "stop"):
          drive_man.stop()
          break

      bool = check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.6)
      if(bool):
        print("Blocked main")
        map.plot(heading,pos)
        goal = None
        continue
      #line following
      inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man, inter_lvl, heading, pos,map)

      #smart-line-following
      bool = False
      if (blocked):
        bool = check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.15)
        if(not bool):
          inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
        else:
          turning("right_spin", drive_man, ir_man)
          map.set_blocked(pos[0], pos[1], heading)
          heading = (heading - 4) % 8
          inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
          if(blocked):
            while True:
              blocked_tup = ultrasound_sensor.read()
              if(blocked_tup != None and blocked_tup > 0.5):
                break
            inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
          else:
            inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
      
      #state_updating
      if(not dead_end and not bool):
        map.add_road(pos[0], pos[1], heading, STATUS.CONNECTED)
        map.add_road(pos[0], pos[1], (heading-1) % 8, STATUS.NONEXISTENT)
        map.add_road(pos[0], pos[1], (heading+1) % 8, STATUS.NONEXISTENT)
        pos = calc_move(pos, heading)
        map.add_intersection(pos[0], pos[1])
        map.add_road(pos[0], pos[1], (heading - 4) % 8, STATUS.CONNECTED)
        map.add_road(pos[0], pos[1], (heading - 5) % 8, STATUS.NONEXISTENT)
        map.add_road(pos[0], pos[1], (heading - 3) % 8, STATUS.NONEXISTENT)
      pull_forward(drive_man, map, heading, pos, ir_man)
      map.plot(heading, pos)

        


    drive_man.stop()
    io.stop()
    filename = 'graph.pickle'
    # Save the map to file.
    print("Saving the map to %s..." % filename)
    with open(filename, 'wb') as file:
      pickle.dump(map, file)
  except BaseException as ex:
    print(f"exception {ex}")
    traceback.print_exc()
    ultrasound_sensor.shutdown()
    drive_man.stop()
    io.stop()
