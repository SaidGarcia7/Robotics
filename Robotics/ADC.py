import pigpio
import sys
import time
import traceback

class ADC():
  def __init__(self, io, pin_l,pin_a, pin_r, bits):
    io.set_mode(pin_l, pigpio.OUTPUT)
    io.set_mode(pin_a, pigpio.OUTPUT)
    io.set_mode(pin_r, pigpio.INPUT)
    self.io = io
    self.pin_l = pin_l
    self.pin_a = pin_a
    self.pin_r = pin_r
    self.bits = bits
    for pin in bits:
      io.set_mode(pin, pigpio.INPUT)
    

  def read(self, address):
    self.io.write(self.pin_l, 0)
    self.io.write(self.pin_a, address)
    self.io.write(self.pin_l, 1)
    self.io.write(self.pin_l, 0)
    self.io.write(self.pin_l, 1)
    while True:
      if(self.io.read(self.pin_r) == 1):
        break

    bs = ""
    for pin in self.bits:
      bs = str(self.io.read(pin)) + bs
    return int(bs, 2)

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  angle_sensor = ADC(io, 27, 4, 17, [9,10,11,12,22,23,24,25])
  while True:
    i = input()
    if(i == "y"):
      print(angle_sensor.read(0))
      print(angle_sensor.read(1))
    elif(i == "n"):
      print("Stopping...")
      break

  io.stop()
