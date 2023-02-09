"""
All custom constants needed for our programs
"""
from vex import *


class Color:
    """
    Any colors needed in the program
    """
    RED = 7
    YELLOW = 8
    BLUE = 9


class AutonomousTask:
    """
    All the possible autonomous tasks
    """
    SCORE_IN_LOW_GOAL = 1
    PUSH_IN_DISKS_WITH_PLOW = 2
    SPIT_OUT_DISKS_WITH_INTAKE = 3
    SHOOT_PRELOAD_LEFT = 4
    SHOOT_PRELOAD_RIGHT = 5
    ROLLER_LEFT = 6
    ROLLER_RIGHT = 7
    ROLLER_BOTH = 8
    SKILLS = 9
    DRIVETRAIN_TEST = 10
    DO_NOTHING = 11
