def fullrobot(shared):
  # Initialize inside the thread.
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  ir_man = IR_man(io, 14, 15, 18)
  drive_man = Drive_Manager(io, 7, 8, 5, 6)
  angle_sensor = AngleSensor(io, 27, 4, 17, [9, 10, 11, 12, 22, 23, 24, 25])
  try:
    while True:
        shared.acquire()
        active = shared.active
        heading = shared.heading
        pos = shared.pos
        goal = shared.goal
        map = shared.map
        shared.release()

        if not active:
            # stop robot
            time.sleep(0.1)
            continue

        # Implementing the herding behavior
        t_prev = time.time()
        inter_lvl = 0.5 
        turns = []
        dead_end = False
        try:
            inter_lvl, heading, dead_end = line_following(drive_man, ir_man, inter_lvl, heading, pos)
            goal = find_goal(map)
            shared.heading = heading
            map.add_intersection(pos[0], pos[1])
            pull_forward(drive_man, map, heading, pos, ir_man)
            manual = False
            exploring = False

            while active:
                shared.acquire()
                if goal is not None and goal.get_coords() != shared.goal.get_coords():
                    goal = shared.goal
                    djkstra(map, goal)

                if goal is None:
                    while not shared.action:
                        shared.release()
                        time.sleep(0.1)
                        shared.acquire()

                desired = None
                at_goal = False
                curr = map.get_intersection(pos[0], pos[1])
                if goal is not None and not manual:
                    if curr.get_dist() == 0:
                        print("Reached Goal")
                        at_goal = True
                        goal = None
                    elif curr is not None:
                        desired = curr.get_dir()
                        if desired is not None:
                            direction = calc_turn(desired, heading)
                        else:
                            print("Input Direction:")
                            while True:
                                direction = input()
                                if direction in INTERSECTION:
                                    break
                    else:
                        print("Error: Current position does not have an intersection.")
                        break
                else:
                    print("Input Direction:")
                    while True:
                        direction = input()
                        if direction in INTERSECTION:
                            break

                if direction == "right_spin" or direction == "left_spin":
                    if desired is not None:
                        while heading != desired:
                            a_init = angle_sensor.read()
                            turns.append(turning(direction, drive_man, ir_man))
                            straighten_out(drive_man, ir_man)
                            a = angle_sensor.read()
                            true_a = true_angle(a, a_init, direction)
                            old_heading = heading
                            heading = calc_heading(heading, true_a, direction)
                            DNE_roads(map, old_heading, heading, direction, pos)
                    else:
                        a_init = angle_sensor.read()
                        turns.append(turning(direction, drive_man, ir_man))
                        straighten_out(drive_man, ir_man)
                        a = angle_sensor.read()
                        true_a = true_angle(a, a_init, direction)
                        old_heading = heading
                        heading = calc_heading(heading, true_a, direction)
                        DNE_roads(map, old_heading, heading, direction, pos)
                elif direction == "stop":
                    drive_man.stop()
                    break

                inter_lvl, heading, dead_end = line_following(drive_man,
                                                              ir_man, inter_lvl, heading, pos)

                if not dead_end:
                      map.add_road(pos[0], pos[1], heading, STATUS.CONNECTED)
                      map.add_road(pos[0], pos[1], heading - 1, STATUS.NONEXISTENT)
                      map.add_road(pos[0], pos[1], heading + 1, STATUS.NONEXISTENT)
                      pos = calc_move(pos, heading)
                      map.add_intersection(pos[0], pos[1])
                      map.add_road(pos[0], pos[1], (heading - 4) % 8, STATUS.CONNECTED)
                      map.add_road(pos[0], pos[1], (heading - 5) % 8, STATUS.NONEXISTENT)
                      map.add_road(pos[0], pos[1], (heading - 3) % 8, STATUS.NONEXISTENT)
                pull_forward(drive_man, map, heading, pos, ir_man)
                map.plot(heading, pos)
                shared.release()
                time.sleep(0.1)

        except:
            drive_man.stop()
            io.stop()

  except BaseException as ex:
    print("Ending Run-Robot due to exception: %s" % repr(ex))
