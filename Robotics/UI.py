from main import *
from Shared import *
import threading
import ctypes
import pickle
from ros import runros

INTERSECTION = ["right_spin", "left_spin", "stop", "straight"]

HEADING = [0,1,2,3,4,5,6,7]

REL_HEADING = {(0, 1):0, (-1, 1):1, (-1, 0):2,
 (-1, -1):3, (0, -1):4, (1, -1):5,
 (1, 0):6, (1, 1):7}

DICT_SO = {(0,1,0) : "stop", (0,0,1) : "right_spin", (0,1,1) : "right_spin", (1,0,0) : "left_spin", (1,1,0) : "left_spin", (1,1,1) : "straight"}

DICT_LINE = {(1,0,1): "straight", (0,0,0): "straight",(0,1,0) : "straight", (0,0,1) : "right_hook", (0,1,1) : "right_turn", (1,0,0) : "left_hook", (1,1,0) : "left_turn", (1,1,1) : "straight"}

DICT_INTERSECTION = {(0,0,0): 0,(0,1,0) : 0, (0,0,1) : 0, (0,1,1) : 0, (1,0,0) : 0, (1,1,0) : 0, (1,1,1) : 1}

DICT_TURN = {(0,0,0): 0,(0,1,0) : 1, (0,0,1) : 1, (0,1,1) : 1, (1,0,0) : 1, (1,1,0) : 1}

DICT_PUSH = {(0,1,0) : 0, (0,0,1) : -1, (0,1,1) : -0.5, (1,0,0) : 1, (1,1,0) : 0.5}

DICT_END = {(0,0,0): 1,(0,1,0) : 0, (0,0,1) : 0, (0,1,1) : 0, (1,0,0) : 0, (1,1,0) : 0, (1,1,1) : 0}

HEADING_DISTANCE = {0: 1, 1: 1.4, 2: 1, 3: 1.4, 4: 1, 5: 1.4, 6: 1, 7: 1.4}
def robot(shared):
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
  dead_end = False
  angle_sensor = AngleSensor(io, 27, 4, 17, [9,10,11,12,22,23,24,25])
  ultrasound_sensor = Ultrasound_man(io, 13, 19,26,16,20,21)
  try:
    map = shared.map
    if(shared.goal is not None):
      goal = map.get_intersection(shared.goal[0],shared.goal[1])
    else:
      goal = None
    heading = shared.heading
    pos = shared.pos
    inter_lvl, heading, dead_end,b = line_following(ultrasound_sensor, drive_man, ir_man, inter_lvl, 
                                                  heading, pos, map)
    shared.heading = heading
    shared.pos = pos
    map.add_intersection(pos[0], pos[1])
    pull_forward(drive_man, map, heading, pos, ir_man)
    map.plot(heading,pos)
    pot_goals = None
    while True:
      while(not shared.action):
        pass
      direction = None
      if(shared.goal is None and shared.direction is None):
        print("Waiting for inputs")
        map.plot(heading,pos)
        while True:
          if(shared.direction is not None):
            direction = shared.direction
            break
          elif(shared.goal is not None):
            break
      while(not shared.action):
        pass
        
      if shared.acquire():
        #ensuring correct state
        if(shared.map_clear):
          map.clear_blocked()
          shared.map_clear = False
        heading = shared.heading
        pos = shared.pos
        direction = shared.direction
        if(shared.map_shift):
          map = shared.map
          shared.map_shift = False
        
        #ensuring correct goal
        if(shared.goal is not None):
          goal = map.get_intersection(shared.goal[0],shared.goal[1])
          shared.directed_explore = False
          # enter directed explore
          if(goal == None):
            shared.directed_explore = True
            pot_goals = map.get_potential_goals(shared.goal)
            if(len(pot_goals) == 0):
              print("clearing")
              map.clear_blocked()
              pot_goals = map.get_potential_goals(shared.goal)
              if(len(pot_goals) == 0):
                print("No path/Fully Explored")
                shared.goal = None
                shared.direction = None
                shared.directed_explore = False
                shared.release()
                continue
              
            path = dir_djkstra(map,pot_goals)
            map.plot(heading, pos)
            curr = map.get_intersection(pos[0], pos[1])
            not_goal = True
            for i in pot_goals:
              if(curr.get_coords() == i.get_coords()):
                not_goal = False
            if(path and curr.get_dir() is None and curr.get_dist() != 0 and not_goal):
                print("All paths blocked or No Path")
                map.clear_blocked()
                dir_djkstra(map, pot_goals)
        
        #running dijkstra's on goal
        if(goal is not None and shared.goal is not None and not shared.exploring):
          path = djkstra(map, goal)
          curr = map.get_intersection(pos[0], pos[1])
          if(path and curr.get_dir() is None and curr.get_dist() != 0):
            print("All paths blocked or No Path")
            map.clear_blocked()
            djkstra(map, goal)
        else:
          goal = None
          map.clear()

        #exploring goal
        desired = None
        at_goal = False
        check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.6, 0.5)
        curr = map.get_intersection(pos[0], pos[1])
        if(shared.exploring):
          if(curr is not None and len(curr.get_unknown()) > 0):
            goal = curr
            djkstra(map, goal)
          else:
            goal = find_goal(map)
            if(goal is None):
              print("Fully Explored")
              shared.direction = None
              shared.goal = None
              shared.release()
              continue
            else:
              path = djkstra(map, goal)
              if(path and curr.get_dir() is None and curr.get_dist() != 0):
                print("All paths blocked")
                map.clear_blocked()
                djkstra(map, goal)
        
        #goal following        
        if(goal is not None and not shared.manual):
          if(curr.get_dist() == 0):
            print("Reached Goal")
            at_goal = True
            goal = None
            if(not shared.exploring):
              shared.direction = None
              shared.goal = None
              shared.release()
              continue
              
          elif curr is not None:
            desired = curr.get_dir()
            if(desired is not None):
              direction = calc_turn(desired, heading, map, pos)
            else:
              if(shared.exploring):
                at_goal = True
          else:
            print("Error: Current position does not have an intersection.")
            break
        
        #directed explore decision
        if(shared.directed_explore):
          dir_djkstra(map, pot_goals)
        if(pot_goals is not None and not shared.manual and shared.directed_explore):
          if curr is not None:
            if(curr.get_coords() == shared.goal):
              print("Reached Goal")
              at_goal = True
              goal = None
              shared.direction = None
              shared.goal = None
              shared.directed_explore = False
              shared.release()
              continue
            desired = curr.get_dir()
            not_goal = True
            for i in pot_goals:
              if(curr.get_coords() == i.get_coords()):
                not_goal = False
            if(desired is not None):
              print("o")
              print(desired)
              direction = calc_turn(desired, heading, map, pos)
            elif(not not_goal):
              direction = decide_turn(shared.goal, curr.get_coords(), heading, map,pos)
              print(direction)
            else:
              shared.release()
              continue
          else:
            print("Error: Current position does not have an intersection.")
            break

        #exploring execution
        check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.6, 0.5)
        if(at_goal and curr is not None and shared.exploring):
          unknown = curr.get_unknown()
          if(len(unknown) > 0):
            desired = unknown.pop(0)
            direction = calc_turn(desired, heading, map, pos)
            
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
              #determining if road exists
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
              shared.goal = None
              shared.direction = None
              shared.release()
              continue
            else:  
              print("new")
              djkstra(map, goal)
            direction = "straight"
            
            
          


        #turning execution
        if(direction == "right_spin" or direction == "left_spin"):
          if(desired is not None):
            if(heading != desired):
              a_init = angle_sensor.read()
              turns.append(turning(direction, drive_man, ir_man))
              straighten_out(drive_man, ir_man)
              a = angle_sensor.read()
              true_a = true_angle(a, a_init, direction)
              drive_man.drive("stop")
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
              direction = None
              shared.direction = direction
          else:
            a_init = angle_sensor.read()
            turns.append(turning(direction, drive_man, ir_man))
            straighten_out(drive_man, ir_man)
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
            direction = None
            shared.direction = direction

        elif(direction == "stop"):
          drive_man.stop()
          break
        
        direction = None
        shared.direction = direction
        shared.pos = pos
        shared.heading = heading
        if(not shared.directed_explore and goal is None):
          shared.goal = goal
        elif(not shared.directed_explore):
          shared.goal = goal.get_coords()
        shared.release()
      else:
        print("skipped")
      
      bool = False
      bool = check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.6, 0.5)
      if(bool):
        print("Blocked main")
        map.plot(heading,pos)
        goal = None
        continue
      #line following
      inter_lvl, heading, dead_end,blocked = line_following(ultrasound_sensor, drive_man, ir_man, inter_lvl, heading, pos, map)

      #smart-line-following
      bool = False
      if(blocked and dead_end):
        while True:
          blocked_tup = ultrasound_sensor.read()
          if(blocked_tup is not None and blocked_tup > 0.5):
              break
        inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
        dead_end = True
      elif(blocked):
        bool = check_ahead(ultrasound_sensor,drive_man, heading, pos, map, 0.15,2)
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
              if(blocked_tup is not None and blocked_tup > 0.5):
                break
            inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
          else:
            inter_lvl, heading, dead_end, blocked = line_following(ultrasound_sensor, drive_man, ir_man,   inter_lvl, heading, pos,map)
      #state_updating
      if(dead_end == False and not bool):
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
      shared.map = map
      shared.pos = pos
      shared.heading = heading
      if shared.acquire():
        print("update")
        shared.robotx = pos[0]
        shared.roboty = pos[1]
        shared.robotheading = heading
        shared.release()


    drive_man.stop()
    io.stop()
  except BaseException as ex:
    print("Ending Run-Robot due to exception: %s" % repr(ex))
    traceback.print_exc()
    drive_man.stop()
    ultrasound_sensor.shutdown()
    io.stop()
    
def UI():
  try:
    pos = (0,0)
    heading = 0
    map = Map()
    print("Would you like to load a map?")
    goal = None
    while True:
      ans = input()
      if(ans == "n"):
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
      elif(ans == "y"):
          with open("graph.pickle", "rb") as f:
            map = pickle.load(f)
            print("What is the goal x:y?(i.e 1:0)")
            goal_input = None
            while (goal_input is None):
              goal_input = input()
            g_list = goal_input.split(":")
            x = int(g_list[0])
            y = int(g_list[1])
            tup = (x,y)
            goal = (x,y)
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
    running = True
    shared = Shared(goal,False,False,map,pos,heading)

    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()
    
    robotthread = threading.Thread(name="RobotThread",target=robot, args=(shared,)) 
    robotthread.start()
    while running:# Grab the user input - implicitly sleep while waiting.
      cmd = None
      cmd = input("Command? ")
      if(cmd is not None and shared.acquire()):
        print(cmd)
        if(cmd == "quit"):
          ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(robotthread.ident),     
          ctypes.py_object(KeyboardInterrupt))
          robotthread.join()
          ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(rosthread.ident),
                                                     ctypes.py_object(KeyboardInterrupt))
          rosthread.join()
          break
        elif(cmd == "pause"):
          print("Waiting")
          shared.action = False
        elif(cmd == "step"):
          shared.action = True
          shared.release()
          if shared.acquire():
            shared.action = False
            shared.release()
        elif(cmd == "resume"):
          print("Resuming")
          shared.action = True
        elif(cmd == "left"):
          shared.direction = "left_spin"
          shared.goal = None
          shared.manual = True
          shared.exploring = False
          shared.directed_explore = False
        elif(cmd == "straight"):
          shared.direction = "straight"
          shared.manual = True
          shared.exploring = False
          shared.goal = None
          shared.directed_explore = False
        elif(cmd == "right"):
          shared.direction = "right_spin"
          shared.manual = True
          shared.exploring = False
          shared.goal = None
          shared.directed_explore = False
        elif(cmd == "explore"):
          shared.exploring = True
          shared.manual = False
          shared.goal = shared.pos
          shared.directed_explore = False
        elif(cmd == "goal"):
          print("What is the goal x:y?(i.e 1:0)")
          goal_input = None
          while (goal_input is None):
            goal_input = input()
          print(goal_input)
          g_list = goal_input.split(":")
          x = int(g_list[0])
          y = int(g_list[1])
          shared.goal = (x,y)
          shared.manual = False
          shared.exploring = False
        elif(cmd == "show"):
          shared.map.plot(shared.heading,shared.pos)
        elif(cmd == "pose"):
          print("What is the start x:y:h?(i.e 1:0:0)")
          s_input = None
          while (s_input is None):
            s_input = input()
          s_list = s_input.split(":")
          x = int(s_list[0])
          y = int(s_list[1])
          shared.heading = int(s_list[2])
          shared.pos = (x, y)
        elif(cmd == "save"):
          filename = '1.pickle'
          # Save the map to file.
          print("Saving the map to %s..." % filename)
          with open(filename, 'wb') as file:
            pickle.dump(shared.map, file)
        elif(cmd == "load"):
          shared.map_shift = True
          with open("1.pickle", "rb") as f:
            shared.map = pickle.load(f)
        elif(cmd == "clear"):
          shared.map_clear = True
        else:
          print("Invalid command")
        shared.release()
        
  except BaseException as ex:
    print("Ending Run-Robot due to exception: %s" % repr(ex))
    
if __name__ == '__main__':
  UI()
  
