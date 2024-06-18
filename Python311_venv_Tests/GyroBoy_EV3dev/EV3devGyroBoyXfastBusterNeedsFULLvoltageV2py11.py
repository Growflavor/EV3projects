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
from ev3fast import *    # source: https://github.com/QuirkyCort/ev3dev-lang-python-fast
# Initialize the EV3 brick.
#ev3 = EV3Brick()

image_awake = Image.open(r'./images_sound/awake2.png')
image_sleeping = Image.open(r'./images_sound/sleeping.png')
image_knocked_out = Image.open(r'./images_sound/knocked_out.png')
image_pinched_middle = Image.open(r'./images_sound/pinched_middle.png')


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


# Initialize the timers.
fall_timer = StopWatch()
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
action_timer = StopWatch()

start_talk = 0
# The following (UPPERCASE names) are constants that control how the program
# behaves.

GYRO_CALIBRATION_LOOP_COUNT = 200
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms
ARM_MOTOR_SPEED = 50  # now -100 to 100% was deg/s

# Actions will be used to change which way the robot drives.
Action = namedtuple('Action', ['drive_speed', 'steering'])

# These are the pre-defined actions
DO_PRESENTATION = Action(drive_speed=1, steering=0)
STOP = Action(drive_speed=0, steering=0)
FORWARD_FAST = Action(drive_speed=80, steering=0)
FORWARD_SLOW = Action(drive_speed=20, steering=0)
BACKWARD_FAST = Action(drive_speed=-75, steering=0)
BACKWARD_SLOW = Action(drive_speed=-10, steering=0)
TURN_RIGHT = Action(drive_speed=10, steering=70)
TURN_LEFT = Action(drive_speed=10, steering=-70)

# The colors that the color sensor can detect are mapped to actions that the
# robot can perform.
ACTION_MAP = {
    color_sensor.COLOR_RED: STOP,
    color_sensor.COLOR_GREEN: FORWARD_FAST,
    color_sensor.COLOR_BLUE: TURN_RIGHT,
    color_sensor.COLOR_YELLOW: TURN_LEFT,
    color_sensor.COLOR_WHITE: BACKWARD_FAST,
    color_sensor.COLOR_BROWN: DO_PRESENTATION,
}


# This function monitors the color sensor and ultrasonic sensor.
#
# It is important that no blocking calls are made in this function, otherwise
# it will affect the control loop time in the main program. Instead, we yield
# to the control loop while we are waiting for a certain thing to happen like
# this:
#
#     while not condition:
#         yield
#
# We also use yield to update the drive speed and steering values in the main
# control loop:
#
#     yield action
#

def talk():
    spkr.speak('Hello I am GyroBoy.')
    # spkr.speak('I am running Python 3.11 on EV3dev Debian Buster Linux Operating System.',espeak_opts='-a 175 -s 150')
    # spkr.speak('I like to experiment!',espeak_opts='-a 175 -s 150')
 

def update_action():
    arm_motor.reset()
    action_timer.restart()

    # Drive forward for 4 seconds to leave stand, then stop.
    left_motor.command = "run-direct"
    right_motor.command = "run-direct"
    yield FORWARD_SLOW
    while action_timer.value_ms < 4000:
        yield

    action = STOP
    yield action

    # Start checking sensors on arms. When specific conditions are sensed,
    # different actions will be performed.
    while True:
        # First, we check the color sensor. The detected color is looked up in
        # the action map.
        new_action = ACTION_MAP.get(color_sensor.color)

        # If the color was found, beep for 0.1 seconds and then change the
        # action depending on which color was detected.
        if new_action is not None:
            action_timer.restart()
            #ev3.speaker.beep(1000, -1)
            #spkr.play_tone(800, 0.1, volume=75, play_type=1)
            #spkr.beep('-f 750 -I 100 -r 3', play_type=1)
            #while action_timer.value_ms < 100:
            #    yield
            #ev3.speaker.beep(0, -1)

            # If the new action involves steering, combine the new steering
            # with the old drive speed. Otherwise, use the entire new action.
            if new_action.steering != 0:
                action = Action(drive_speed=action.drive_speed,
                                steering=new_action.steering)
            else:
                action = new_action
                if new_action.drive_speed == 1:
                    start_talk = 1
            yield action

        # If the measured distance of the ultrasonic sensor is less than 250
        # millimeters, then back up slowly.
        if ultrasonic_sensor.distance_centimeters < 25: # Or 250mm
            # Back up slowly while wiggling the arms back and forth.
            yield BACKWARD_SLOW
            
            #(ARM_MOTOR_SPEED, 30, wait=False)
            arm_motor.on_for_degrees(ARM_MOTOR_SPEED, 30, block=False)
            while arm_motor.is_running:
                yield
            arm_motor.on_for_degrees(ARM_MOTOR_SPEED, -60, block=False)
            while arm_motor.is_running:
                yield
            arm_motor.on_for_degrees(ARM_MOTOR_SPEED, 30, block=False)
            while arm_motor.is_running:
                yield

            # Randomly turn left or right for 4 seconds while still backing
            # up slowly.
            turn = random.choice([TURN_LEFT, TURN_RIGHT])
            yield Action(drive_speed=BACKWARD_SLOW.drive_speed,
                         steering=turn.steering)
            action_timer.restart()
            while action_timer.value_ms < 4000:
                yield

            # Beep and then restore the previous action from before the
            # ultrasonic sensor detected an obstruction.
            action_timer.restart()
            #ev3.speaker.beep(1000, -1)
            #while action_timer.value_ms < 100:
            #    yield
            #ev3.speaker.beep(0, -1)
            #spkr.beep('-f 750 -I 100 -r 3', play_type=1)
            #spkr.play_tone(200, 0.1, volume=75, play_type=1)
            yield action

        # This adds a small delay since we don't need to read these sensors
        # continuously. Reading once every 100 milliseconds is fast enough.
        action_timer.restart()
        while action_timer.value_ms < 100:
            yield


# If we fall over in the middle of an action, the arm motors could be moving or
# the speaker could be beeping, so we need to stop both of those.
def stop_action():
    #ev3.speaker.beep(0, -1)
    arm_motor.stop()

talk()
while True:
    # Sleeping eyes and light off let us know that the robot is waiting for
    # any movement to stop before the program can continue.
    #ev3.screen.load_image(ImageFile.SLEEPING)
    #if color_sensor.color == color_sensor.COLOR_BLACK:
    #    do_presentation = 1

    # if start_talk == 1:
    #    left_motor.stop()
    #    right_motor.stop()
    #    start_talk = 0
    #    lcd.image.paste(image_pinched_middle, (0, 0))
    #    lcd.update()
    #    my_leds.set_color('LEFT', 'YELLOW')
    #    my_leds.set_color('RIGHT', 'YELLOW')
    #    spkr.speak('Ready to speak!', play_type=1)
    #    talk()
    # else:
    lcd.image.paste(image_sleeping, (0, 0))
    lcd.update()
    my_leds.all_off()

   # spkr.speak('Hallo, ich bin hier.', play_type=1)
 
   # Reset the sensors and variables.
    left_motor.reset()
    right_motor.reset()
    # gyro_sensor.mode = 'GYRO-CAL'  # calibrate to 0 twice
    # gyro_sensor.mode = 'GYRO-CAL' 
    # gyro_sensor.mode = 'GYRO-ANG'
    motor_position_sum = 0
    wheel_angle = 0
    motor_position_change = [0, 0, 0, 0]
    drive_speed, steering = 0, 0
    control_loop_count = 0
    robot_body_angle = -0.25

    # Since update_action() is a generator (it uses "yield" instead of
    # "return") this doesn't actually run update_action() right now but
    # rather prepares it for use later.
    action_task = update_action()

    # Calibrate the gyro offset. This makes sure that the robot is perfectly
    # still by making sure that the measured rate does not fluctuate more than
    # 2 deg/s. Gyro drift can cause the rate to be non-zero even when the robot
    # is not moving, so we save that value for use later.

    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.rate
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            time.sleep((5) / 1000)
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    # Awake eyes and green light let us know that the robot is ready to go!
    spkr.play_file('./images_sound/speed_up.wav', volume=50, play_type=1)
    #ev3.screen.load_image(ImageFile.AWAKE)
    lcd.image.paste(image_awake, (0, 0))
    lcd.update()
    #spkr.speak('Hello!', play_type=1)

    my_leds.set_color('LEFT', 'GREEN')
    my_leds.set_color('RIGHT', 'GREEN')

    # Main control loop for balancing the robot.
    fall_timer.restart()
    while True:
        # This timer measures how long a single loop takes. This will be used
        # to help keep the loop time consistent, even when different actions
        # are happening.
        single_loop_timer.restart()

        # This calculates the average control loop period. This is used in the
        # control feedback calculation instead of the single loop time to
        # filter out random fluctuations.
        if control_loop_count == 0:
            # The first time through the loop, we need to assign a value to
            # avoid dividing by zero later.
            average_control_loop_period = TARGET_LOOP_PERIOD / 1000
            control_loop_timer.restart()
        else:
            # average_control_loop_period = (control_loop_timer.value_ms() / 1000 / control_loop_count) -- values do not use ()
            average_control_loop_period = (control_loop_timer.value_ms / 1000 / control_loop_count)
        control_loop_count += 1

        # calculate robot body angle and speed
        gyro_sensor_value = gyro_sensor.rate
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        robot_body_angle += robot_body_rate * average_control_loop_period

        # calculate wheel angle and speed
        left_motor_angle = left_motor.position
        right_motor_angle = right_motor.position
        previous_motor_sum = motor_position_sum
        motor_position_sum = left_motor_angle + right_motor_angle
        change = motor_position_sum - previous_motor_sum
        motor_position_change.insert(0, change)
        del motor_position_change[-1]
        wheel_angle += change - drive_speed * average_control_loop_period
        wheel_rate = sum(motor_position_change) / 4 / average_control_loop_period

        # This is the main control feedback calculation.
        output_power = (-0.01 * drive_speed) + (0.8 * robot_body_rate +
                                                15 * robot_body_angle +
                                                0.08 * wheel_rate +
                                                0.12 * wheel_angle)
        if output_power > 100:
            output_power = 100
        if output_power < -100:
            output_power = -100

        # Drive the motors.
        # https://ev3dev-lang.readthedocs.io/projects/python-ev3dev/en/ev3dev-jessie/motors.html
        # COMMAND_RUN_DIRECT = 'run-direct'
        # Run the motor at the duty cycle specified by duty_cycle_sp. Unlike other run commands, changing duty_cycle_sp while running will take effect immediately.
        left_motor.duty_cycle_sp=(output_power - 0.1 * steering)
        right_motor.duty_cycle_sp=(output_power + 0.1 * steering)
        # left_motor.command = "run-direct"
        # right_motor.command = "run-direct"

        # Check if robot fell down. If the output speed is +/-100% for more
        # than one second, we know that we are no longer balancing properly.
        if abs(output_power) < 100:
            fall_timer.restart()
        elif fall_timer.value_ms > 1000:    
            # elif fall_timer.value_ms > 1000 or start_talk == 1:
            break

        # This runs update_action() until the next "yield" statement.
        action = next(action_task)
        if action is not None:
            drive_speed, steering = action

        # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power
        # calculation above depends on having a certain amount of time in each
        # loop.
        s1 = single_loop_timer.value_ms
        if s1 < 15:
            time.sleep((TARGET_LOOP_PERIOD - s1) / 1000)
            # print(s1)

    # Handle falling over. If we get to this point in the program, it means
    # that the robot fell over.

    # Stop all of the motors.
    stop_action()
    left_motor.stop()
    right_motor.stop()

    # Knocked out eyes and red light let us know that the robot lost its
    # balance.
    my_leds.set_color('LEFT', 'RED')
    my_leds.set_color('RIGHT', 'RED')

    #ev3.screen.load_image(ImageFile.KNOCKED_OUT)
    lcd.image.paste(image_knocked_out, (0, 0))
    lcd.update()
    spkr.speak('Ouch!')

    spkr.play_file('./images_sound/speed_down.wav', volume=50, play_type=1)

    # Wait for a few seconds before trying to balance again.
    time.sleep(3)
