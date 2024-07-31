from motor import *
import pigpio
import sys
import time
import traceback

drive_pathways = {
    "straight": (0.845, 0.87),
    "right_veer": (0.87, 0.845),
    "left_veer": (0.755, 0.9),
    "right_steer": (0.9, 0.78),
    "left_steer": (0.72, 0.95),
    "right_turn": (0.9, 0.65),
    "left_turn": (0.55, 0.95),
    "right_hook": (0.87, 0),
    "left_hook": (0, 0.92),
    "left_spin": (-0.92, 0.87),
    "right_spin": (0.8,-0.9),
    "stop": (0,0)
}
class Drive_Manager():
  def __init__(self, io, pin_1a, pin_1b, pin_2a, pin_2b):
    motor_1 = Motor(io, pin_1a, pin_1b)
    motor_2 = Motor(io, pin_2a, pin_2b)
    self.io = io
    self.motor_1 = motor_1
    self.motor_2 = motor_2

  def drive(self, angle):
    (x, y) = drive_pathways[angle]
    self.motor_1.set_level(x)
    self.motor_2.set_level(y)

  def stop(self):
    self.motor_1.stop()
    self.motor_2.stop()
    
  def flower_power(self):
      for i in drive_pathways:
        self.drive(i)
        time.sleep(4.25)
        self.drive("stop")
        while True:
            x = input()
            if(x == "y"):
                print("Moving on....")
                time.sleep(1)
                break
  
  def pwm(self, l,r):
    self.motor_1.set_level(l)
    self.motor_2.set_level(r)

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  d = Drive_Manager(io, 7, 8, 5, 6)
  d.drive("straight")
  time.sleep(4)
  d.stop()
  io.stop()
