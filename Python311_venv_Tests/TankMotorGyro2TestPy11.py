#!/home/robot/venv311/bin/python3.11

# Import the necessary libraries
import time
import math
#from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
#from ev3dev2.sensor.lego import *
from ev3dev2.motor import OUTPUT_A, OUTPUT_B, LargeMotor, MoveTank, SpeedPercent, follow_for_ms, FollowGyroAngleErrorTooFast
from ev3dev2.sensor.lego import GyroSensor

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
#tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
#steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
#radio = Radio()

#color_sensor_in1 = ColorSensor(INPUT_1)
#ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
#gyro_sensor_in3 = GyroSensor(INPUT_3)
#gps_sensor_in4 = GPSSensor(INPUT_4)
#pen_in5 = Pen(INPUT_5)

#motorC = LargeMotor(OUTPUT_C) # Magnet

# Here is where your code starts


# Instantiate the MoveTank object
tank = MoveTank(OUTPUT_A, OUTPUT_B)

# Initialize the tank's gyro sensor
tank.gyro = GyroSensor()

# Calibrate the gyro to eliminate drift, and to initialize the current angle as 0
tank.gyro.calibrate()

def tankx(speed, angle, tms):
    tank.follow_gyro_angle(
        kp=11.3, ki=0.05, kd=3.2,
        speed=SpeedPercent(speed),
        target_angle=angle,
        follow_for=follow_for_ms,
        ms=tms
    )



for i in range(8):
    x = ((i+1)*45)
    tankx(30, x, 3500)


