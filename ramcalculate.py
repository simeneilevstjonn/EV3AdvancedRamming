#!/usr/bin/env pybricks-micropython
# © Simen Eilevstjønn 2021. All rights reserved.
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import cos, sin, degrees, radians, sqrt, acos
from coloursafedriving import ColourDriver
from random import randint

# RamCalculator class
# Provides methods to calculate the position of an opponent, given the object is moving along a constant line. Uses the gyroscope for accurate angle measurements.
class RamCalculator:
    # Constructor
    def __init__(self, drive_base, ultrasonic_sensor, gyroscope, colour_sensor, speed, radius):
        """
        Creates an instance of the RamCalculator class

        :param DriveBase drive_base: The instance of the DriveBase class to use.
        :param UltrasonicSensor ultrasonic_sensor: The instance of the UltrasonicSensor class to use.
        :param GyroSensor gyroscope: The instance of the GyroSensor class to use.
        :param ColorSensor colour_sensor: The instance of the ColorSensor class to use.
        :param int Speed: The ramming speed to use. Specified in millimetres per second.
        :param int Radius: The radius of the fighting arena. Specified in millimetres.
        """

        # Store all variables
        self.driveBase = drive_base
        self.ultrasonicSensor = ultrasonic_sensor
        self.gyroscope = gyroscope
        self.colourSensor = colour_sensor
        self.speed = speed
        self.radius = radius

        # Create an instance of ColourDriver
        self._colourDriver = ColourDriver(drive_base, colour_sensor, Color.BLACK)
    
    # Rotate until an object is located in front within set distance
    def getObjectDist(self, distance_threshold, rotation_threshold, rotate_step=10, step_delay=1) -> tuple:
        """
        Rotates the robot, until an object is located in front within the distance_threshold. Returns None if it rotates more than rotation_threshold.

        :param float distance_threshold: The maximum distance in which to detect an object. Specified in millimetres.
        :param float rotation_threshold: The maximum rotation without any detected object before None is returned. Specified in degrees.
        :param int rotate_step: The degrees turned between each measurement. Specified in degrees. Default is 10. Lower will yield more accuracy, though more delay.
        :param int step_delay: The delay between each measurement. Specified in milliseconds. Default is 10.
        :return: A tuple with degrees turnt and distance to the detected object. 
        """

        # Reset the gyroscope
        self.gyroscope.reset_angle(0)

        # Loop while we have not turnt more than the threshold
        while self.gyroscope.angle() < rotation_threshold:
            # Measure distance
            distance = self.ultrasonicSensor.distance()

            # Check if distance is less than threshold
            if distance < distance_threshold:
                # Return degrees turnt and distance
                return (self.gyroscope.angle(), distance)
            
            # Else turn and wait
            self.driveBase.turn(-rotate_step) # Must be negative, since DriveBase uses clockwise as positive direction.
            wait(step_delay)
        
        # If loop breaks, no object has been detected. Return angles turnt and None.
        return (self.gyroscope.angle(), None)

    # Calulcate optimal attack angle change for ramming opponent
    def getAttackAngle(self, a, b, angle, time) -> float:
        """
        Gets the optimal angle change in order to ram the opponent using the set speed, given two measurements of distance with an exact angle as well as time between.

        :param float a: The distance between the robot and the opponent during the first measurement. Specified in millimetres.
        :param float b: The distance between the robot and the opponent during the second measurement. Specified in millimetres.
        :param float angle: The angle between the two vectors from the robot's position to the measured points A and B. Specified in degrees.
        :param float time: The time between the two measurements a and b. Specified in milliseconds.
        :rtype: float
        :return: Returns a a course change in degrees between -180 and 180 degrees.
        """

        # Define the vector function for the opponent's path
        def o(t) -> tuple:
            """
            The movement path for the opponent, given a constant speed and course.

            :param float t: The time from the first measurement was taken. Specified in milliseconds.
            :rtype: tuple
            :return: A unit vector to the point the opponent should be in after t milliseconds from the initial measurement. Coordinates are specified in millimetres.
            """
            
            return (((b * cos(radians(angle)) - a) / time) * t + a, ((b * sin(radians(angle))) / time) * t)

        # Find the t value for the first possible intersect point between the robot and the opponent.
        intersect_time = - (time * b) / (b - sqrt(a**2 * (cos(radians(angle)))**2 - a**2 + (self.speed / 1000)**2 * time**2) - a * cos(radians(angle)))

        # Get the intersect point unit vector
        c = o(intersect_time + time)

        # Calculate the dot product of the two vectors RC (c) and RB (b)
        dotp = c[0] * b * cos(radians(angle)) + c[1] * b * sin(radians(angle))

        # Calculate the angle difference using arccos
        angle_diff = degrees(acos(dotp / (sqrt(c[0]**2 + c[1]**2) * sqrt((b * cos(radians(angle)))**2 + (b * sin(radians(angle)))**2))))

        # Since the angle is the shortest angle between the two vectors, regardless of what is actually positive direction, 
        # the angle should be multiplied by the sign of the difference between the y value for the intersect point and the y value of b
        
        # Math does not have a sign function, having to use if-else instead
        return angle_diff if (c[1] - a * cos(radians(angle)) >= 0) else -angle_diff

    # Excecute a ramming attack
    def doAttack(self, distance_threshold) -> bool:
        """
        Excecutes the ramming attack using the parameters supplied.

        :param float distance_threshold: The maximum distance from the 
        :rtype: bool
        :return: Returns True if an attack was successfully initiated. Returns False if no opponent was detected within the detection zone.
        """

        # Do the first scan. Use maximum 360 degrees of rotation. The other parameters might need to be tweaked.
        scan_a = self.getObjectDist(distance_threshold, 360)

        # Save the time when scan has completed. Create stopwatch.
        stopwatch = StopWatch()
        

        # Return false if the scan did not find anything.
        if scan_a[1] is None:
            return False
        
        # Wait for some time before executing the second scan. The delay time might have to be tweaked.
        wait(500)

        # Do the second scan. Use maximum 360 degrees of rotation. The other parameters might need to be tweaked.
        scan_b = self.getObjectDist(distance_threshold, 360)

        # Save the time when scan has completed. 
        time_interval = stopwatch.time()

        # Return false if the scan did not find anything.
        if scan_b[1] is None:
            return False

        # The angle between the two vectors should be between -180 and 180 degrees
        angle = scan_b[0] if scan_b[0] < 180 else scan_b[0] - 360

        # Calculate the optimum angle change
        change = self.getAttackAngle(scan_a[1], scan_b[1], angle, time_interval)

        # Turn to the calculated angle
        self.driveBase.turn(-change) # Must be negative, since DriveBase uses clockwise as positive direction.

        # Drive using safe drive. When it hits line, reverse for radius / 2, then turn to a random angle between 45 and 180 degrees.
        self._colourDriver.drive(self.speed, self.radius / 2, randint(45, 180))

        # Mind there will be a delay not accounted for, due to the time needed for turning and acceleration. Hopefully, it is insignificant, if not, a delay will have to be introduced.

        # Return true to signal the attack has been attempted
        return True

