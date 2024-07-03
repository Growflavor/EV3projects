#!/home/robot/venv37/bin/python3
# We will do the imports required for this notebook here

from ev3dev2.sound import Sound
spkr = Sound()
spkr.play_tone(2000, 0.3, volume=75, play_type=0)
from ev3dev2.stopwatch import StopWatch
timer = StopWatch()

timer.restart()
import spatialmath as sm
print('sm: ' + str(timer.value_ms))

timer.restart()
import spatialmath.base as smb
print('smb: ' + str(timer.value_ms))
spkr.play_tone(600, 0.3, volume=75, play_type=0)

timer.restart()
import spatialgeometry as sg
print('sg: ' + str(timer.value_ms))

timer.restart()
from swift import Swift
print('Swift: ' + str(timer.value_ms))


print('# numpy provides import array and linear algebra utilities')
timer.restart()
import numpy as np
print('np: ' + str(timer.value_ms))
#from numpy import array as np_array
#from numpy import zeros as np_zeros
spkr.play_tone(1500, 0.3, volume=75, play_type=0)



print('# The matrix exponential method')
timer.restart()
from scipy.linalg import expm, logm
print('scipy: ' + str(timer.value_ms))
spkr.play_tone(1000, 0.3, volume=75, play_type=0)



print('# sympy provides symbolic mathematic operations') 
timer.restart()
import sympy as sym
print('sym: ' + str(timer.value_ms))

print('the robotics toolbox provides robotics specific functionality')
timer.restart()
import roboticstoolbox as rtb
print('rtb: ' + str(timer.value_ms))

print('module import complete')

# spatial math provides objects for representing transformations
# spkr.speak('import spatial math')
#import spatialmath as sm
#timer.restart()
#from spatialmath import Twist3 as sm_Twist3
#print('sm_Twist3: ' + str(timer.value_ms))
#spkr.play_tone(800, 0.3, volume=75, play_type=0)

# import spatialmath.base as smb
# timer.restart()
# from spatialmath.base import skewa as smb_skewa
# print('smb_skewa: ' + str(timer.value_ms))
# spkr.play_tone(600, 0.3, volume=75, play_type=0)

# timer.restart()
# from spatialmath.base import transl as smb_transl
# print('smb_transl: ' + str(timer.value_ms))
# spkr.play_tone(400, 0.3, volume=75, play_type=0)

# sympy provides symbolic mathematic operations 
# import sympy as sym

# import spatialgeometry as sg

# from swift import Swift

spkr.speak('start calculations')
print('Define the symbolic variables')
v1, v2, v3, eta1, eta2, eta3 = sym.symbols(['upsilon_1', 'upsilon_2', 'upsilon_3', 'eta_1', 'eta_2', 'eta_3'])

print('Define the twist')
s = np.array([v1, v2, v3, eta1, eta2, eta3])
print(s)
spkr.play_tone(100, 0.3, volume=75, play_type=0)

print('Turn the twist into an se(3) matrix')
timer.restart()
s_sk = smb.skewa(s)
print(s_sk)
print('smb_skewa: ' + str(timer.value_ms))
spkr.play_tone(200, 0.3, volume=75, play_type=0)

# Do this using code
s = np.zeros(6)
print(s)

print('Convert to an augmented skew-symmetric matrix')
timer.restart()
s_sk = smb.skewa(s)
print(s_sk)
print('smb_skewa: ' + str(timer.value_ms))
spkr.play_tone(300, 0.3, volume=75, play_type=0)

print('Convert to an SE(3) matrix')
timer.restart()
T = expm(s_sk)
print(T)
print('expm: ' + str(timer.value_ms))
spkr.play_tone(400, 0.3, volume=75, play_type=0)

print('Define the symbolic variables')
theta = sym.symbols('theta')

# Do this using code
s = np.array([0, 0, 0, 0, 0, theta])
print(s)

print('Convert to an augmented skew-symmetric matrix')
timer.restart()
s_sk = smb.skewa(s)
print(s_sk)
print('smb_skewa: ' + str(timer.value_ms))
spkr.play_tone(500, 0.3, volume=75, play_type=0)

print('Convert to an SE(3) matrix')
timer.restart()
T = sym.simplify(sym.Matrix(s_sk).exp())
print(T)
print('sym_simplify, matrix: ' + str(timer.value_ms))
spkr.play_tone(600, 0.3, volume=75, play_type=0)

print('Define the symbolic variables')
timer.restart()
x, y, z = sym.symbols(['x', 'y', 'z'])
print('sym_symbols: ' + str(timer.value_ms))

# Do this using code
s = np.array([x, y, z, 0, 0, 0])
print(s)

print('Convert to an augmented skew-symmetric matrix')
timer.restart()
s_sk = smb.skewa(s)
print(s_sk)
print('smb_skewa: ' + str(timer.value_ms))

spkr.play_tone(700, 0.3, volume=75, play_type=0)

print('Convert to an SE(3) matrix')
timer.restart()
T = sym.simplify(sym.Matrix(s_sk).exp())
print(T)
print('sym_simplify, matrix: ' + str(timer.value_ms))
spkr.play_tone(800, 0.3, volume=75, play_type=0)

axis = [0.0, 0.0, 1.0]
point = [2.0, 3.0, 2.0]
pitch = 0.5

timer.restart()
S = sm.Twist3.UnitRevolute(axis, point, pitch)
print(S)
print('sm_Twist3: ' + str(timer.value_ms))
spkr.play_tone(900, 0.3, volume=75, play_type=0)

print('Describe the coordinate frame {x} relative to the world frame')
timer.restart()
Tx = smb.transl(3, 4, 2)
print(Tx)
print('smb_transl: ' + str(timer.value_ms))
spkr.play_tone(1000, 0.3, volume=75, play_type=0)

spkr.speak('Test complete.')
print(' Start your own methods.')