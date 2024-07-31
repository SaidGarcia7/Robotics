
import pigpio
import time

class Ultrasound():
  def __init__(self, io, pintrig, pinecho):
    self.io = io
    self.pintrig = pintrig
    self.pinecho = pinecho

    io.set_mode(pintrig, pigpio.OUTPUT)
    io.set_mode(pinecho, pigpio.INPUT)
    
    cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
    cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

    self.last_trigger_time = 0

  def trigger(self):
    current_time = time.time()

    if current_time - self.last_trigger_time >= 0.05:

      self.io.write(self.pintrig, 1)
      self.io.write(self.pintrig, 0)

      self.last_trigger_time = current_time

  def rising(self, pin, level, ticks):
    self.rising_ticks = ticks


  def falling(self, pin, level, ticks):
    fall_ticks = ticks

    deltatick = fall_ticks - self.rising_ticks
    if (deltatick < 0):
      deltatick += 2 ** 32

    flight_time = deltatick

    speed_of_sound = 343 / 1000000 

    dist = (flight_time * speed_of_sound) / 2

    self.dist = dist

  def read(self):
    if hasattr(self, "dist"):
      return self.dist
    else: 
      return None

if __name__ == '__main__':
  io = pigpio.pi()
  if not io.connected:
    print("Unable to connection to pigpio daemon!")
    sys.exit(0)
  print("GPIO ready...")
  m = Ultrasound(io, 19, 20)
  m.trigger()
  last_trigger_time = time.time()
  while True:
    # Re-trigger every 50ms.
    now = time.time()
    if now - last_trigger_time > 0.050:
      m.trigger()
      last_trigger_time = now
    
    print(m.read())
    
    
    
