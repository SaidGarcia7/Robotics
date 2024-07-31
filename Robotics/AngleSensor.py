import pigpio
import sys
import time
import traceback
from ADC import *
from math import *
import math
from drive import *
import matplotlib.pyplot as plt


class AngleSensor():
  def __init__(self, io, pin_l,pin_a, pin_r, bits):
    self.io = io
    self.ADC = ADC(io, pin_l,pin_a, pin_r, bits)

  def read(self):
    mx = self.ADC.read(0)
    mx = (2 * ((mx - 78)/116)) - 1
    my = self.ADC.read(1)
    my = (2 * ((my - 43)/119)) - 1
    rad = atan2(my,mx)
    return rad * (180/math.pi)
    

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  mx = []
  my = []
  angle_sensor = AngleSensor(io, 27, 4, 17, [9,10,11,12,22,23,24,25])
  d = Drive_Manager(io, 7, 8, 5, 6)
  t = time.time()
  while time.time() - t < 7:
    d.drive("right_spin")
    tup = angle_sensor.read()
    print(tup)
    #mx.append(tup[0])
    #my.append(tup[1])
  d.drive("stop")
  #print(min(mx))
  #print(min(my))
  #print(max(mx))
  #print(max(my))
  io.stop()
