#!/usr/bin/env pybricks-micropython
# © Simen Eilevstjønn 2021. All rights reserved.
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Colour safe driving, allows for driving, and stopping at the outer line.
class ColourDriver:
    # Constructor
    def __init__(self, drive_base, colour_sensor, watch_colour):
        """
        Constructs an instance of the ColourDriver class.

        :param DriveBase drive_base: The instance of the DriveBase class to use.
        :param ColorSensor colour_sensor: The instance of the ColorSensor class to use.
        :param Color colour: The colour to watch out for.
        """

        # Store all variables
        self.driveBase = drive_base
        self.colourSensor = colour_sensor
        self.watchColour = watch_colour

    # Drive at a constant speed
    def drive(speed, reverse_distance=300, turn_angle=180, distance_limit=float('inf')):
        """
        Starts driving at a constant speed, until the colour sensor detects the watch colour.

        :param int speed: The speed to drive at. Specified in millimetres per second.
        :param int reverse_distance: The distance to reverse after the watch colour has been detected. Specified in millimetres. Default 300mm.
        :param int turn_angle: The angle to turn after having reversed. Specified in degrees. Default 180 degrees.
        :param float distance_limit: The maximum distance to drive, before stopping. Specified in millimetres. Default is no limit.
        """

        # Set speed settings for reversing. Hopefully this works without using the full settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration) overload.
        self.driveBase.settings(speed)

        # Reset the accumulated driven length
        self.driveBase.reset()

        # Start driving at the given speed
        self.driveBase.drive(speed, 0)

        # Constantly check wether the colour is not the watch colour
        while (self.colourSensor.color() != self.watchColour) AND (self.driveBase.distance() < distance_limit):
            # Wait for some short period, and check again. It is important that there is no chance that a line 10mm wide will be missed due to the delay.
            wait(8)
        
        # When loop has broken, either the colour has been detected or the distance limit has been exceeded. 
        # Check if distance is exceeded
        if self.driveBase.distance() >= distance_limit:
            # Break robot. Hopefully reversing will quicken the decerelation as opposed to stop(). If not, another method incorporating straight() must be made.
            self.driveBase.straight(-reverse_distance/10)
            self.driveBase.robot.stop()

            # Break function
            return

        # If we are here, the colour has been detected. Must reverse.
        self.driveBase.straight(-reverse_distance)

        # Turn to the specified angle
        self.driveBase.turn(-turn_angle) # Must be negative, since DriveBase uses clockwise as positive direction.