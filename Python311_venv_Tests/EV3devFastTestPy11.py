#!/home/robot/venv311/bin/python3.11

from time import perf_counter
# from ev3dev.ev3 import *
from ev3fast import *

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
