#!/home/robot/py11venv/bin/python3.11

from collections import namedtuple
import random
import time
from PIL import Image

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
from ev3fast import *
# Initialize the EV3 brick.
#ev3 = EV3Brick()

#image_awake = Image.open(r'./images_sound/awake2.png')
#image_sleeping = Image.open(r'./images_sound/sleeping.png')
#image_knocked_out = Image.open(r'./images_sound/knocked_out.png')
image_pinched_middle = Image.open(r'./GyroBoy_ev3dev2/images_sound/pinched_middle.png')


spkr = Sound()
lcd = Display()
my_leds = Leds()

# Initialize the motors connected to the drive wheels.
left_motor = LargeMotor('outD')
right_motor = LargeMotor('outA')

# Initialize the motor connected to the arms.
arm_motor = MediumMotor('outC')

# Initialize the Color Sensor. It is used to detect the colors that command
# which way the robot should move.
color_sensor = ColorSensor('in1')

# Initialize the gyro sensor. It is used to provide feedback for balancing the
# robot.
gyro_sensor = GyroSensor('in2')

# Initialize the gyro sensor. It is used to provide feedback for balancing the
# robot.
touch_sensor = TouchSensor('in3')

# Initialize the ultrasonic sensor. It is used to detect when the robot gets
# too close to an obstruction.
ultrasonic_sensor = UltrasonicSensor('in4')

#color
# Color detected by the sensor, categorized by overall value.
COLOR_LIST = ['No color', 'Black', 'Blue', 'Green', 'Yellow', 'Red', 'White', 'Brown']

while True:
    my_leds.set_color('LEFT', 'AMBER')
    my_leds.set_color('RIGHT', 'AMBER')
    lcd.image.paste(image_pinched_middle, (0, 0))
    lcd.update()
    spkr.speak(COLOR_LIST[color_sensor.color])
    print(color_sensor.color)
    if ultrasonic_sensor.distance_centimeters < 5:
        break

