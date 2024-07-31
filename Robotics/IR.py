import pigpio
import sys
import time
import traceback


class IR():
  def __init__(self, io, pin):
    io.set_mode(pin, pigpio.INPUT)
    self.io = io
    self.pin = pin

  def read(self):
    return self.io.read(self.pin)

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")
  ir = IR(io, 18)
  i = 0
  while i < 10:
    print(ir.read())
    time.sleep(1)
    i += 1
