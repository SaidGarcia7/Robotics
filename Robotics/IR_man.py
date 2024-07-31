from IR import *
import pigpio
import sys
import time
import traceback

class IR_man():
  def __init__(self, io, pin_l, pin_m, pin_r):
    self.io = io
    self.ir_l = IR(io, pin_l)
    self.ir_m = IR(io, pin_m)
    self.ir_r = IR(io, pin_r)

  def read(self):
    return (self.ir_l.read(), self.ir_m.read(), self.ir_r.read())