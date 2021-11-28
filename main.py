#!/usr/bin/env pybricks-micropython
# © Simen Eilevstjønn 2021. All rights reserved.
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from ramcalculate import RamCalculator
from coloursafedriving import ColourDriver
from random import randint

"""
Code style:
Class names in PascalCase
Parameters and other variables within functions in snake_case
Methods and properties in camelCase
File names in lowercase

All names in en_UK, even when library is in en_US.
"""

# Arena radius. Must be measured. Using 600mm as a placeholder
radius = 600

# Define robot objects
# Brick
ev3 = EV3Brick()

# Motors
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)

# Sensors
ultrasonicSensor = UltrasonicSensor(Port.S1)
gyroscope = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE) # Anticlockwise is positive direction for angles
colourSensor = ColorSensor(Port.S3)

# Drive base
# Wheel diameter and axle track must be thoroughly measured and calibrated.
driveBase = DriveBase(leftMotor, rightMotor, 58, 120)

# ColourDriver and RamCalculator
colourDriver = ColourDriver(driveBase, colourSensor, Color.BLACK)
ramCalculator = RamCalculator(driveBase, ultrasonicSensor, gyroscope, colourSensor, 600, radius)
# Speed has not been tested, must test to validate robot can achieve and rapidly break.


# The attack procedure follows:

# Start by moving slowly to line, and turn, as per the rules.
colourDriver.drive(200, radius / 3) 

# Follow this attack structure indefinetly
while True:
    # Attempt a ram attack
    if ramCalculator.doAttack(radius): # Distance_threshold currently set to radius. Must see how this works.
        # If attack was successfully initiated, drive for radius / 2 and retry without turning
        colourDriver.drive(200, radius / 3, randint(45, 120) * (1 if randint(0, 1) else -1), radius / 2)
    else:
        # If unsuccessfull, drive for radius * 0.8
        colourDriver.drive(200, radius / 3, randint(45, 120) * (1 if randint(0, 1) else -1), radius * 0.8)

        # Turn a random angle between 45 and 120 degrees in either direction
        driveBase.turn(randint(45, 120) * (1 if randint(0, 1) else -1))

