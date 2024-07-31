from Ultrasound import *
import threading
import pigpio

class Ultrasound_man():
  def __init__(self, io, pin_l_trig, pin_m_trig, pin_r_trig,
               pin_l_echo, pin_m_echo, pin_r_echo):
    self.io = io
    self.ultrasound_l = Ultrasound(io, pin_l_trig, pin_l_echo)
    self.ultrasound_m = Ultrasound(io, pin_m_trig, pin_m_echo)
    self.ultrasound_r = Ultrasound(io, pin_r_trig, pin_r_echo)
    print("Starting triggering thread...")
    self.triggering = True
    self.thread = threading.Thread(name="TriggerThread", target=self.run)
    self.thread.start()
    time.sleep(0.1)

  
  def trigger(self):
    self.ultrasound_l.trigger()
    self.ultrasound_m.trigger()
    self.ultrasound_r.trigger()
  
  def run(self):
    while self.triggering:
      self.trigger()
      time.sleep(0.2)

  def shutdown(self):
    self.triggering = False
    print("Waiting for triggering thread to finish...")
    self.thread.join()
    print("Triggering thread returned.")
  
  def read(self):
    return (self.ultrasound_m.read())

def main():
  io = pigpio.pi()
  if not io.connected:
    print("Unable to connection to pigpio daemon!")
    sys.exit(0)
  print("GPIO ready...")

  pin_l_trig = 13
  pin_m_trig = 19
  pin_r_trig = 26
  pin_l_echo = 16
  pin_m_echo = 20
  pin_r_echo = 21

  ultrasound_sensor = Ultrasound_man(io, pin_l_trig, pin_m_trig, pin_r_trig,
                                     pin_l_echo, pin_m_echo, pin_r_echo )

  try:
    while True:
      ultrasound_sensor.trigger()
      time.sleep(0.050)
      distances = ultrasound_sensor.read()
      print(f'Distances = {distances}')

  except:
    io.stop()

if __name__ == '__main__':
  main()  
