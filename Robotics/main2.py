from Ultrasound_man import *
from drive import *
import pigpio


def herding():
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

  drive_man = Drive_Manager(io, 7, 8, 5, 6)


  # Start by triggering.
  ultrasound_sensor.trigger()
  last_trigger_time = time.time()
  # Loop from here.
  while True:
    # Re-trigger every 50ms.
      
    # Read the latest measurements (having ‘‘arrived’’ in the last 50ms).

    # Take action.
    (left_ultra, middle_ultra, right_ultra) = ultrasound_sensor.read()
    #print(left_ultra)
    #print(middle_ultra)
    #print(right_ultra)
    
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
      drive_man.drive("right_spin")
    elif (left_ultra < 0.2 and middle_ultra < 0.2 and right_ultra < 0.2):
      drive_man.drive("stop")
    elif (left_ultra > 0.2 and middle_ultra < 0.2 and right_ultra < 0.2):
      drive_man.drive("left_spin")

def wall_following():
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

  drive_man = Drive_Manager(io, 7, 8, 5, 6)


  # Start by triggering.
  ultrasound_sensor.trigger()
  last_trigger_time = time.time()
  # Loop from here.
  wall = 0
  print("Which wall are you following?")
  while True:
    ans = input()
    if ans == "l":
      wall = 0
      break
    if ans == "r":
      wall = 2
      break
  
  pwm_l_global = 0.845
  pwm_r_global = 0.87
  while True:
    # Re-trigger every 50ms.
    now = time.time()
    if now - last_trigger_time > 0.050:
      ultrasound_sensor.trigger()
      last_trigger_time = now
      
    # Read the latest measurements (having ‘‘arrived’’ in the last 50ms).

    # Take action.
    tup = ultrasound_sensor.read()
    print(tup)
    if(tup[wall] is not None):
      if(tup[1] < 0.2):
        drive_man.drive("stop")
      elif(tup[wall] <= 0.4 and tup[wall] >= 0.2):
        if(wall == 1):
          pwm_l = pwm_l_global + ((0.3 - tup[wall])) * (0.8)
          pwm_r = pwm_r_global - ((0.3 - tup[wall])) * (0.9)
        else:
          pwm_l = pwm_l_global - ((0.3 - tup[wall])) * (0.8)
          pwm_r = pwm_r_global + ((0.3 - tup[wall])) * (0.9)
        drive_man.pwm(pwm_l,pwm_r)
      else:
        drive_man.stop()
      

if __name__ == '__main__':
  herding()
    
      


      
      
      
