import threading
import ctypes
import time
import pickle
import pigpio
import sys
from main2 import *

class Shared_Herding():
  def __init__(self):
    self.shared12 = []
    
               


def runui(shared1):
  try:
    while True:
      command = input("Enter command (i.e. run/stop/quit): ").strip()
      if command == 'run':
          shared1.shared12 = [1]
          print("Robot running.")
      elif command == 'stop':
          shared1.shared12 = [1, 2]
          print("Robot stopping.")
      elif command == 'quit':
          shared1.shared12 = [1, 2, 3]
          print("Exiting program.")
          break
      else:
        print("Invalid command.")

  except BaseException as ex:
     print("Ending Run-UI due to exception: %s" % repr(ex))


def herdingrobot(io, shared1):
  pin_l_trig = 13
  pin_m_trig = 19
  pin_r_trig = 26
  pin_l_echo = 16
  pin_m_echo = 20
  pin_r_echo = 21 

  ultrasound_sensor = Ultrasound_man(io, pin_l_trig, pin_m_trig, pin_r_trig,
                                     pin_l_echo, pin_m_echo, pin_r_echo )

  drive_man = Drive_Manager(io, 7, 8, 5, 6)


  # Start by triggering.
  ultrasound_sensor.trigger()
  last_trigger_time = time.time()
  # Loop from here.
  try:
    while True:
    # Re-trigger every 50ms.

    # Read the latest measurements (having ‘‘arrived’’ in the last 50ms).

    # Take action.
      (left_ultra, middle_ultra, right_ultra) = ultrasound_sensor.read()
    #print(left_ultra)
    #print(middle_ultra)
    #print(right_ultra)
    
      if(len(shared1.shared12) == 1):
        pass

      while (len(shared1.shared12) == 2):
        drive_man.stop()
      
      while (len(shared1.shared12) == 3):
        drive_man.stop()
        io.stop

      if (left_ultra == None or middle_ultra == None or right_ultra == None):
        drive_man.drive("stop")
      elif (left_ultra > 0.2 and middle_ultra > 0.2 and right_ultra > 0.2):
        drive_man.drive("straight")
      elif (left_ultra < 0.2 and middle_ultra > 0.2 and right_ultra > 0.2):
        drive_man.drive("right_turn")
      elif (left_ultra > 0.2 and middle_ultra > 0.2 and right_ultra < 0.2):
        drive_man.drive("left_turn")
      elif (left_ultra < 0.2 and middle_ultra > 0.2 and right_ultra < 0.2):
        drive_man.drive("straight")
      elif (left_ultra > 0.2 and middle_ultra < 0.2 and right_ultra > 0.2):
        drive_man.drive("stop")
      elif (left_ultra < 0.2 and middle_ultra < 0.2 and right_ultra > 0.2):
        drive_man.drive("right_hook")
      elif (left_ultra < 0.2 and middle_ultra < 0.2 and right_ultra < 0.2):
        drive_man.drive("stop")
      elif (left_ultra > 0.2 and middle_ultra < 0.2 and right_ultra < 0.2):
        drive_man.drive("left_hook")



      
  
  except BaseException as ex:
    print("Ending Run-Robot due to exception: %s" % repr(ex))

  ultrasound_sensor.shutdown()
  drive_man.stop()
  



# Main function
def main():
  io = pigpio.pi()
  if not io.connected:
    print("Unable to connection to pigpio daemon!")
    sys.exit(0)
  print("GPIO ready...")
    
  shared1 = Shared_Herding()
  # Immediately start the robot worker thread - notice the arguments.
  robotthread = threading.Thread(name="RobotThread",target=herdingrobot, args=(io, shared1,))
  robotthread.start()

  # Do the UI work.
  runui(shared1)
  
  # End the robot thread (send the KeyboardInterrupt exception).
  ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(robotthread.ident),     
  ctypes.py_object(KeyboardInterrupt))
  robotthread.join()
  io.stop()


if __name__ == "__main__":
  main()
