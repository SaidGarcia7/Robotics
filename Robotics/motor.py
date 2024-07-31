import pigpio
import sys
import time
import traceback

pin_1a = 7
pin_1b = 8

pin_2a = 5
pin_2b = 6
class Motor():

  def __init__(self, io, pin_a, pin_b):
    self.io = io
    self.pin_a = pin_a
    self.pin_b = pin_b

    io.set_mode(pin_a, pigpio.OUTPUT)
    io.set_mode(pin_b, pigpio.OUTPUT)

    io.set_PWM_range(pin_a, 255)
    io.set_PWM_range(pin_b, 255)

    io.set_PWM_frequency(pin_a, 1000)
    io.set_PWM_frequency(pin_b, 1000)

    self.stop()

  def stop(self):
    self.set_level(0)


  def set_level(self, level):
    if(1>= level >= 0):
      self.io.set_PWM_dutycycle(self.pin_a, level * 255)
      self.io.set_PWM_dutycycle(self.pin_b, 0)
    if(-1<= level < 0):
      self.io.set_PWM_dutycycle(self.pin_a, 0)
      self.io.set_PWM_dutycycle(self.pin_b, level * -255)

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
      print("Unable to connection to pigpio daemon!")
      sys.exit(0)
  print("GPIO ready...")

  motor_1 = Motor(io, pin_1a, pin_1b)
  motor_2 = Motor(io, pin_2a, pin_2b)

  i =0
  while i < 4:
    motor_1.stop()
    motor_2.stop()
    motor_1.set_level(0.85)
    motor_2.set_level(0.845)
    time.sleep(4.25)
    motor_1.stop()
    motor_2.stop()
    motor_1.set_level(0.75)
    time.sleep(1.7)
    i +=1

  motor_1.stop()
  motor_2.stop()
  io.stop()
