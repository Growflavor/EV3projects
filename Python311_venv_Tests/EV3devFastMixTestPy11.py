#!/home/robot/py11venv/bin/python3.11

from time import perf_counter
#from ev3dev.ev3 import *
#from collections import namedtuple
#import random
#import time
#from PIL import Image

#from pybricks.hubs import EV3Brick
#from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor, GyroSensor, TouchSensor
#from pybricks.parameters import Port, Color, ImageFile, SoundFile
#from pybricks.tools import wait, StopWatch
from ev3dev2.stopwatch import StopWatch
from ev3dev2.sound import Sound
from ev3dev2.button import Button
#from ev3dev2.sensor import *
#from ev3dev2.motor import OUTPUT_A, OUTPUT_D, OUTPUT_C, LargeMotor, MediumMotor
#from ev3dev2.sensor.lego import ColorSensor, TouchSensor, UltrasonicSensor, GyroSensor
from ev3dev2.led import Leds
from ev3dev2.display import Display
from ev3fast import *    # source: https://github.com/QuirkyCort/ev3dev-lang-python-fast
# Initialize the EV3 brick.
#ev3 = EV3Brick()

#image_awake = Image.open(r'./images_sound/awake2.png')
#image_sleeping = Image.open(r'./images_sound/sleeping.png')
#image_knocked_out = Image.open(r'./images_sound/knocked_out.png')


spkr = Sound()
lcd = Display()
my_leds = Leds()


lMotor = LargeMotor('outA')
rMotor = LargeMotor('outD')
lSensor = ColorSensor('in1')
rSensor = ColorSensor('in4')

LOOPS = 1000

startTime = perf_counter()
for a in range(0,LOOPS):
  valueL = lSensor.raw
  valueR = rSensor.raw
  totalL = (valueL[0] + valueL[1] + valueL[2])
  totalR = (valueR[0] + valueR[1] + valueR[2])
  error = totalL - totalR
  lMotor.speed_sp = 200 + error
  rMotor.speed_sp = 200 - error
  lMotor.run_forever()
  rMotor.run_forever()
endTime = perf_counter()

lMotor.stop()
rMotor.stop()

print(str(LOOPS / (endTime - startTime)))