#!/home/robot/py11venv/bin/python3.11

import time
from ev3dev2.sound import Sound
# from ev3dev2.motor import *
# from ev3dev2.sound import Sound
from ev3dev2.button import Button
# from ev3dev2.sensor import *
# from ev3dev2.sensor.lego import *
from ev3dev2.led import Leds

btn = Button()
spkr = Sound()
my_leds = Leds()

start_time = 0
curr2 = 0

def speech(text, times):
    #curr2 = time.time()
    #while (times - 3) >= round(curr2  - start_time):
    #	curr2 = time.time()
    spkr.speak(text,espeak_opts='-a 175 -s 150')
    time.sleep(1) 
	

my_leds.set_color('LEFT', 'RED')
my_leds.set_color('RIGHT', 'RED')

spkr.speak('ready! Press right button at exactly 3 PM 24 seconds')

start_time = time.time()

spkr.speak('Hello, I am the eclipse totality guide and I will be guiding you trough this eclipse.')

speech('T minus 25 minutes', 5*60)
speech('T minus 20 minutes', 10*60)
speech('T minus 15 minutes, look for Venus', 15*60)
speech('T minus 10 minutes, switch to white light filter', 20*60)
speech('look for Jupiter and Sirius',21*60)
speech('binoculars: look for Orion belt stars',24*60)
speech('binoculars: look for pleiades & hayades clusters',25*60)
speech('binoculars: look for comet', 26*60)
speech('look for snow snakes', 28*60)
speech("T minus 1 minute: Baileys Beades with Sunglasses and remove scope filters.", 29*60)
speech('Diamond ring',(29*60)+40)
speech('Totality! focus & corona in Black scope',30*60)
speech('plus 20 seconds, look at dark surroundings & look for stars while waiting for scope & binocular time', (30*60)+20)
speech('plus 40 seconds, Xavien see pink prominances in bino scope', (30*60)+40) 
speech('1 minute, person 1 Black scope',(30*60)+60)
speech('1 minute 20',(30*60)+80)
speech('Mid eclipse! Mid Eclipse! person 2 Black scope', (30*60)+90)
speech('1 minute 45', (30*60)+105)
speech('1 minute 55, person 3 Black scope',(30*60)+115)
speech('2 minutes 15, ',(30*60)+135)
speech('2 minutes 30, Take FINAL Look at Prominances!',(30*60)+150)
speech('2 minutes 45, 15 seconds left!',(30*60)+165)
speech('Totality ended, Diamond ring & Baileys beades', (30*60)+180)
speech("Look for snow snakes",(30*60)+190)
speech('I hope you had a nice eclipse!',(30*60)+220)
	