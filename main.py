"""
Competition Code for VRC: Spin-Up (2022-2023)
Author: Derek Baier (deekb on GithHub)
Project homepage: https://github.com/deekb/VexCode
Project archive https://github.com/deekb/VexCode/archive/master.zip
Version: 6.5.0
Availible for use and modification under the MIT liscense
Contact Derek.m.baier@gmail.com for more information
"""
# <editor-fold desc="Imports and liscense">
from vex import *

__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up 2022-2023"
__url__ = "https://github.com/deekb/VexCode"
__download_url__ = "https://github.com/deekb/VexCode/archive/master.zip"
__version__ = "0.1.4_stable"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "MIT"
# </editor-fold>


brain = Brain()


class PIDMotor:
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = PIDMotor(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the speed if it has not yet reached the desired speed
    :param kd: Kd value for the PID: Reduces the speed of response and limit overshoot
    :param t: Time between PID updates
    """

    def __init__(self, motor_object, kp: float = 0.4, kd: float = 0.05, t: float = 0.01):
        self.motor_object = motor_object
        self.kp = kp  # adjust these
        self.kd = kd  # adjust these
        self.t = t  # amount of time between cycles (seconds)
        self.e_pr = 0
        self.target_v = 0
        self.v = 0
        self.e = 0
        self.d = 0
        self.o = 0

    def PID_update(self) -> None:
        """
        Update the PID state with the most recent motor and target velocities and send the normalized value to the motor
        """
        self.v = abs(self.velocity(PERCENT))
        self.e = self.target_v - self.v
        self.d = (self.e - self.e_pr) / self.t
        self.o = self.kp * self.e + self.kd * self.d
        self.e_pr = self.e
        self.motor_object.set_velocity(self.v + self.o, PERCENT)

    def PID_loop(self):
        """
        Used to run in a thread using "Thread(<motor to run PID on>.PID_loop)"; it updates the values the PID uses and
        handles applying new normalized speed values to the motor
        :return: None
        """
        while True:
            self.PID_update()
            wait(self.t, SECONDS)

    def set_velocity(self, velocity, unit):
        """
        Set the motors targer velocity using the PID, make sure you run PID_loop in a new thread or this
        will have no effect
        :param velocity: The new target velocity of the motor
        :param unit: The velocity unit - only PERCENT is currently supported
        """
        if unit == PERCENT:
            self.target_v = velocity
        else:
            raise NotImplementedError("Unit not implemented: \"" + str(unit) + "\"")

    def set_stopping(self, **kwargs):
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.set_stopping(kwargs)

    def stop(self):
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.stop()

    def spin(self, direction):
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.spin(direction)

    def velocity(self, *args):
        """
        Passthrough method to the "Motor" class
        """
        return self.motor_object.velocity(*args)


class Motors:
    """
    A class containing references to all motors and motor groups attatched to the robot including PID enabled motors
    """
    # Motors:
    leftFrontMotor = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    leftRearMotor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
    rightFrontMotor = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
    rightRearMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
    roller = Motor(Ports.PORT19, GearSetting.RATIO_36_1, False)
    flywheel = PIDMotor(Motor(Ports.PORT10, GearSetting.RATIO_36_1, True), kp=0.4, kd=0.05, t=0.01)
    intake = Motor(Ports.PORT13, GearSetting.RATIO_36_1, True)
    # Motor groups:
    left = MotorGroup(leftFrontMotor, leftRearMotor)
    right = MotorGroup(rightFrontMotor, rightRearMotor)
    allWheels = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)
    # PID handler threads:
    Thread(flywheel.PID_loop)


class Sensors:
    """
    A class that contains references to all sensors and other input devices attatched to the robot
    """
    inertial = Inertial(Ports.PORT2)
    controller = Controller(PRIMARY)
    optical_roller = Optical(Ports.PORT14)
    optical_disk = Optical(Ports.PORT8)
    ultrasonic = Sonar(brain.three_wire_port.g)


class Constants:
    """"
    A class to store custom constants to make the code more readable and make sure that all references to constants
    point to a differernt object
    """
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
    FORWARD = 5
    REVERSE = 6
    RED = 7
    YELLOW = 8
    BLUE = 9
    PURPLE = 10
    ROLLER = 11
    SHOOTER = 12
    TANK = 13
    ARCADE = 14
    COAST = 15
    BRAKE = 16
    HOLD = 17
    YES = 18
    NO = 19
    ON = 20
    OFF = 21
    AUTONOMOUS = 22
    DRIVER = 23
    ENABLED = 24
    DISABLED = 25
    ROLL_ROLLER_SMART = 26
    PUSH_IN_DISKS_WITH_PLOW = 27
    SPIT_OUT_DISKS_WITH_INTAKE = 28
    SHOOT_PRELOAD = 29
    ROLL_ROLLER = 30
    BOTH_ROLLERS = 31
    PI = 3.141592


class Globals:
    """
    Stores variable that may need to be (or ought to be able to be) accessed by any function in the program, here you can set default/initial values for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc,
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    WHEEL_RADIUS_MM = 50
    WHEEL_CIRCUMFERENCE_MM = 2 * WHEEL_RADIUS_MM * Constants.PI
    ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM = 125
    AUTONOMOUS_TASK = None
    HEADING_OFFSET_TOLERANCE = 1  # How many degrees off is "Close enough"
    CALIBRATION_RESET_DPS_LIMIT = 5  # How many degrees per second does the inertial sensor have to report to invalidate and restart calibration
    DELTA_HEADING_SAMPLE_RATE_MS = 50  # Take a delta_heading sample every X Milliseconds, or None to disable delta_heading sampling
    CONTROL_MODE = None
    COMPETITION_STATE = None
    TEAM = None
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    STOPPING_MODE = None
    FLYWHEEL_ACTIVE = False
    ROLLER_ACTIVE = False
    INTAKE_ACTIVE = False
    DISK_READY = False
    PAUSE_LOADING_THREAD = False
    STRING_UNLOAD_ACTIVE = False


# <editor-fold desc="Print/Clear functions">
def bprint(string) -> None:
    """
    Print a string to the brain screen
    :param string: the string to print to the screen
    """
    brain.screen.print(string)
    brain.screen.next_row()


def bclear() -> None:
    """
    Clears the brain screen
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


def cprint(string) -> None:
    """
    Print a string to the controller screen
    :param string: the string to print to the screen
    """
    Sensors.controller.screen.print(string)
    Sensors.controller.screen.next_row()


def cclear() -> None:
    """
    Clears the controller screen
    """
    Sensors.controller.screen.clear_screen()
    Sensors.controller.screen.set_cursor(1, 1)


# </editor-fold>


def cubic_normalize(value: float, linearity: float = Globals.SPEED_CURVE_LINEARITY) -> float:
    """
    Normalize a value across a cubic curve with a linearity
    :param linearity: How close to a lienar function the normalizer should use
    :type linearity: float
    :param value: The value to normalize
    :type value: float
    :rtype: float
    :returns: The passed value normalized across a cubic curve
    """
    return value ** 3 + linearity * value / (1 + linearity)


def controller_input_to_motor_power(controller_l: tuple, controller_r: tuple) -> tuple:
    """
    Convert raw input from a controller into left and right motor output values
    :param controller_l: The left stick's (X, Y) pair
    :type controller_l: tuple[float, float]
    :param controller_r: The left stick's (X, Y) pair
    :type controller_r: tuple[float, float]
    :rtype: tuple[float, float]
    :returns: A tuple of (left_motor_power, right_motor_power)
    :exception: AssertionError
    """
    left_speed, right_speed = (None, None)  # Ensure that if Globals.CONTROL_MODE is invalid we catch it
    controller_l_x, controller_l_y = controller_l
    controller_r_x, controller_r_y = controller_r
    if Globals.CONTROL_MODE == Constants.TANK:
        # Only normalize the values we are going to use
        controller_l_y = cubic_normalize(controller_l_y() * 0.01, Globals.SPEED_CURVE_LINEARITY)
        controller_r_y = cubic_normalize(controller_r_y() * 0.01, Globals.SPEED_CURVE_LINEARITY)
        left_speed, right_speed = controller_l_y, controller_r_y
    if left_speed is None and right_speed is None:
        raise RuntimeError("Left and right speeds never initialized, Check Globals.CONTROL_MODE")
    return left_speed * -100, right_speed * -100


def move_with_offset(left_motor, right_motor, left_offset: float = 0, right_offset: float = 0) -> None:
    """
    Move with the controller input using the passed offsets
    :param right_motor: The motor to use as the right side
    :type right_motor: Motor || PIDmotror
    :param left_motor: The motor to use as the left side
    :type left_motor: Motor || PIDmotror
    :param left_offset: The offset to apply to the left motor (defaults to 0)
    :type left_offset: float
    :param right_offset: The offset to apply to the right motor (defaults to 0)
    :type right_offset: float
    """
    left_speed, right_speed = controller_input_to_motor_power(
        (Sensors.controller.axis4.position, Sensors.controller.axis3.position),
        (Sensors.controller.axis1.position, Sensors.controller.axis2.position))
    left_motor.set_velocity((left_speed + left_offset), PERCENT)
    right_motor.set_velocity((right_speed + right_offset), PERCENT)


def setup() -> None:
    """
    A setup function, sets the globals to the values selected on the controller using its screen to print them
    """
    settings = (
        ("Team", [("Red", Constants.RED),
                  ("Blue", Constants.BLUE)]),
        ("Stopping", [("Coast", Constants.COAST),
                      ("Brake", Constants.BRAKE)]),
        ("Autonomous", [("Shoot", Constants.SHOOT_PRELOAD),
                        ("Plow disks", Constants.PUSH_IN_DISKS_WITH_PLOW),
                        ("Roller", Constants.ROLL_ROLLER),
                        ("Both rollers", Constants.BOTH_ROLLERS),
                        ("Spit disks", Constants.SPIT_OUT_DISKS_WITH_INTAKE)]),
        ("Control Mode", [("Tank", Constants.TANK)])
    )
    setting_index = 0
    while setting_index < len(settings):
        choice = 0
        setting_name = settings[setting_index][0]
        total_values = len(settings[setting_index][1])
        while Sensors.controller.buttonA.pressing() or Sensors.controller.buttonB.pressing():
            pass
        while not Sensors.controller.buttonA.pressing() and not Sensors.controller.buttonB.pressing():
            value_printable = settings[setting_index][1][choice][0]
            value = settings[setting_index][1][choice][1]
            cclear()
            cprint(setting_name + ": " + value_printable)
            if setting_index == 0:
                Globals.TEAM = value
            elif setting_index == 1:
                Globals.STOPPING_MODE = value
            elif setting_index == 2:
                Globals.AUTONOMOUS_TASK = value
            elif setting_index == 3:
                Globals.CONTROL_MODE = value
            # Wait until a button is pressed
            while not any((Sensors.controller.buttonLeft.pressing(),
                           Sensors.controller.buttonRight.pressing(),
                           Sensors.controller.buttonA.pressing(),
                           Sensors.controller.buttonB.pressing())):
                wait(5)
            if Sensors.controller.buttonRight.pressing() and choice < total_values - 1:
                choice += 1
            elif Sensors.controller.buttonLeft.pressing() and choice > 0:
                choice -= 1
        if Sensors.controller.buttonB.pressing():
            setting_index -= 1
        else:
            setting_index += 1
        # Wait untill all buttons are released
        while any((Sensors.controller.buttonLeft.pressing(),
                   Sensors.controller.buttonRight.pressing(),
                   Sensors.controller.buttonA.pressing(),
                   Sensors.controller.buttonB.pressing())):
            wait(5)


def auton_log(string: str) -> bool:
    """
    Write to the currently running autonomous log stored in Globals.AUTONOMOUS_LOG unless Globals.AUTONOMOUS_LOG is None
    :param string: the string to write
    :rtype: bool
    :returns: True if the function successfully wrote to the file otherwise False
    """
    if Globals.AUTONOMOUS_LOG:
        Globals.AUTONOMOUS_LOG.write("[" + str(brain.timer.time(MSEC)) + "]: " + string + "\n")
        return True
    else:
        return False


# noinspection PyTypeChecker
# ^ Prevents linter from getting mad about reporting an int but returning an Optional[int] with no "typing" library on the vex brain's python version
def get_optical_color(optical_sensor: Optical) -> int:
    """
    Get the color currently detected by the passed optical sensor
    :param optical_sensor: The sensor to read from
    :rtype: int
    :returns: Constants.RED, Constans.BLUE, or None
    """
    if optical_sensor.is_near_object():
        optical_hue = optical_sensor.hue()
        if optical_hue < 30 or optical_hue > 340:
            return Constants.RED
        elif 215 < optical_hue < 255:
            return Constants.BLUE
        elif 28 < optical_hue < 50:
            return Constants.YELLOW
    return


def turn_to_heading(heading, turn_aggression) -> None:
    """
    Turn to an absolute heading using the inertial sensor
    :param heading: The absolute heading to turn to
    :param turn_aggression: The multiplier for delta_heading
    """
    heading = heading % 360
    current_heading = Sensors.inertial.heading(DEGREES) % 360
    left_turn_difference = (current_heading - heading)
    right_turn_difference = (heading - current_heading)
    if left_turn_difference < 0:
        left_turn_difference = left_turn_difference + 360
    if right_turn_difference < 0:
        right_turn_difference = right_turn_difference + 360
    if left_turn_difference < right_turn_difference:
        delta_heading = left_turn_difference
    else:
        delta_heading = right_turn_difference
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.spin(FORWARD)
    delta_heading_samples = []
    delta_heading_last_sample_time = brain.timer.time()
    movement_start_time = brain.timer.time(MSEC)
    while abs(delta_heading) > Globals.HEADING_OFFSET_TOLERANCE:
        if left_turn_difference < right_turn_difference:
            delta_heading = left_turn_difference
            Motors.left.set_velocity(delta_heading * turn_aggression + 5, PERCENT)
            Motors.right.set_velocity((delta_heading * turn_aggression + 5) * -1, PERCENT)
        else:
            delta_heading = right_turn_difference
            Motors.left.set_velocity((delta_heading * turn_aggression + 5) * -1, PERCENT)
            Motors.right.set_velocity(delta_heading * turn_aggression + 5, PERCENT)
        current_heading = Sensors.inertial.heading(DEGREES) % 360
        left_turn_difference = (current_heading - heading)
        right_turn_difference = (heading - current_heading)
        if left_turn_difference < 0:
            left_turn_difference = left_turn_difference + 360
        if right_turn_difference < 0:
            right_turn_difference = right_turn_difference + 360
        if brain.timer.time() - delta_heading_last_sample_time >= Globals.DELTA_HEADING_SAMPLE_RATE_MS:
            delta_heading_samples.append(delta_heading)
            delta_heading_last_sample_time = brain.timer.time()
    Motors.allWheels.stop()
    wait(100)
    current_heading = Sensors.inertial.heading(DEGREES) % 360
    left_turn_difference = (current_heading - heading)
    right_turn_difference = (heading - current_heading)
    if left_turn_difference < 0:
        left_turn_difference = left_turn_difference + 360
    if right_turn_difference < 0:
        right_turn_difference = right_turn_difference + 360
    if left_turn_difference < right_turn_difference:
        delta_heading = left_turn_difference
    else:
        delta_heading = right_turn_difference
    auton_log("[turn_to_heading]: Turned to heading " + str(heading) + " with accuracy of " + str(abs(delta_heading)) + " degrees in " + str(brain.timer.time(MSEC) - movement_start_time) + " ms")
    auton_log("[turn_to_heading]: Memory dump of delta heading samples: " + str(delta_heading_samples))


def move_towards_heading(heading, speed, turn_aggression, distance_mm) -> None:
    """
    Move towards a heading using dynamic course correction
    :param heading: The absolute heading to move towards
    :param speed:  The base speed to move at
    :param turn_aggression: The multiplier for delta_heading
    :param distance_mm: The distance to move before stopping the movement
    """
    initial_speed = speed
    heading = heading % 360
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.spin(FORWARD)
    initial_distance_traveled = (((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                                 / 360) * Globals.WHEEL_CIRCUMFERENCE_MM
    distance_traveled = abs((((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                             / 360) * Globals.WHEEL_CIRCUMFERENCE_MM - initial_distance_traveled)
    delta_heading_samples = []
    delta_heading_last_sample_time = brain.timer.time()
    movement_start_time = brain.timer.time(MSEC)
    while distance_traveled < distance_mm:
        distance_traveled = abs((((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                                 / 360) * Globals.WHEEL_CIRCUMFERENCE_MM - initial_distance_traveled)
        current_heading = Sensors.inertial.heading(DEGREES) % 360
        left_turn_difference = (current_heading - heading)
        right_turn_difference = (heading - current_heading)
        if left_turn_difference < 0:
            left_turn_difference += 360
        if right_turn_difference < 0:
            right_turn_difference += 360
        if left_turn_difference < right_turn_difference:  # Turn towards the most efficient direction
            delta_heading = left_turn_difference
            if abs(delta_heading) > 5:  # if we are too far off course don't move forward, only turn
                speed = 0
            else:
                speed = initial_speed
            Motors.left.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
            Motors.right.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
        else:
            delta_heading = right_turn_difference
            if abs(delta_heading) > 5:  # if we are too far off course don't move forward, only turn
                speed = 0
            else:
                speed = initial_speed
            Motors.left.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
            Motors.right.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
        if brain.timer.time() - delta_heading_last_sample_time >= Globals.DELTA_HEADING_SAMPLE_RATE_MS:
            delta_heading_samples.append(delta_heading)
            delta_heading_last_sample_time = brain.timer.time()
    Motors.allWheels.stop()
    wait(100)
    distance_traveled = abs((((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                             / 360) * Globals.WHEEL_CIRCUMFERENCE_MM - initial_distance_traveled)
    current_heading = Sensors.inertial.heading(DEGREES) % 360
    auton_log("[turn_to_heading]: Moved towards heading " + str(heading) + " with end accuracy of " + str(abs(current_heading - heading)) + " degrees, average accuracy of " + str(sum(delta_heading_samples) / len(delta_heading_samples)) + " and distance accuracy of " + str(distance_traveled - distance_mm) + " mm in " + str(brain.timer.time(MSEC) - movement_start_time) + "ms")
    auton_log("[turn_to_heading]: Memory dump of delta heading samples: " + str(delta_heading_samples))


def roll_roller():
    """
    Roll the roller 90 degrees (only works during autonomout)
    """
    Motors.allWheels.set_stopping(COAST)  # Always good while running into things
    Motors.allWheels.set_velocity(-15, PERCENT)
    Motors.allWheels.spin(FORWARD)
    while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
        wait(5)
    Motors.leftRearMotor.stop()
    Motors.rightRearMotor.stop()
    Motors.leftFrontMotor.set_velocity(-3, PERCENT)
    Motors.rightFrontMotor.set_velocity(-3, PERCENT)
    auton_log("Rolling roller")
    Motors.roller.set_velocity(80)
    Motors.roller.spin_for(REVERSE, 200, DEGREES)
    Motors.allWheels.stop()


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    try:
        Globals.AUTONOMOUS_LOG = open("autonomous.log", "a")  # "a" for append
    except (OSError, AttributeError):  # No SD card present
        Globals.AUTONOMOUS_LOG = None
    bprint("Autonomous thread started")
    if auton_log("----------Autonomous log Open----------"):
        bprint("Log opened")
    else:
        bprint("Loging failed, SD card initialization error")
    if Globals.AUTONOMOUS_TASK == Constants.PUSH_IN_DISKS_WITH_PLOW:
        auton_log("Autonomous will attempt to push a disk or stack of disks into the low goal with the plow")
        auton_log("Moving forward")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
        auton_log("Moving backward")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == Constants.SPIT_OUT_DISKS_WITH_INTAKE:
        auton_log("Autonomous will attempt to push a disk or stack of disks into the low goal with the intake")
        Motors.intake.set_velocity(100, PERCENT)
        auton_log("Reversing loader")
        Motors.intake.spin(REVERSE)
        auton_log("Moving backward 500 mm")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
        auton_log("Moving forward 500 mm")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == Constants.SHOOT_PRELOAD:
        auton_log("Autonomous will attempt to shoot a preload into the high goal from the center of the field")
        auton_log("Reseting inertial sensor to allow correct orientation detection...")
        Sensors.inertial.set_heading(0, DEGREES)  # Reset the sensor
        auton_log("Inertial sensor reset complete")
        auton_log("Turning towards the center of field...")
        turn_to_heading(25, 0.5)
        auton_log("Moving towards the center of the field...")
        turn_to_heading(heading=45, turn_aggression=0.5)
        move_towards_heading(heading=45, speed=50, turn_aggression=1, distance_mm=1525)
        auton_log("Aiming at goal...")
        turn_to_heading(-35, 0.5)
        auton_log("Starting up flywheel")
        Motors.flywheel.set_velocity(80, PERCENT)
        Motors.flywheel.spin(FORWARD)
        auton_log("Waiting for full speed...")
        while Motors.flywheel.velocity(PERCENT) < 60:
            wait(10)
        auton_log("Spinup complete, waiting an aditional 3500 MS...")
        wait(3500)
        auton_log("Done")
        Motors.intake.set_velocity(100, PERCENT)
        auton_log("Firing")
        Motors.intake.spin(FORWARD)
        wait(750)
        auton_log("Unsticking disk")
        Motors.intake.spin(REVERSE)
        wait(400)
        auton_log("Firing")
        Motors.intake.spin(FORWARD)
        wait(2000)
        auton_log("Spinning down")
        Motors.intake.stop()
        Motors.flywheel.stop()
    elif Globals.AUTONOMOUS_TASK == Constants.ROLL_ROLLER:
        auton_log("Autonomous will attempt to roll the roller without the optical sensor")
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == Constants.BOTH_ROLLERS:
        auton_log("Autonomous will attempt to roll the roller without the optical sensor")
        roll_roller()
        Motors.allWheels.set_stopping(BRAKE)
        auton_log("Moving forward")
        move_towards_heading(heading=0, speed=40, turn_aggression=1, distance_mm=100)
        turn_to_heading(heading=35, turn_aggression=0.5)
        move_towards_heading(heading=35, speed=50, turn_aggression=1, distance_mm=1075)
        turn_to_heading(heading=45, turn_aggression=0.5)
        move_towards_heading(heading=45, speed=70, turn_aggression=1, distance_mm=1700)
        auton_log("Turning around")
        turn_to_heading(heading=90, turn_aggression=0.5)
        auton_log("Backing up")
        move_towards_heading(heading=90, speed=40, turn_aggression=1, distance_mm=140)
        auton_log("Turning towards roller")
        turn_to_heading(heading=-135, turn_aggression=0.5)
        move_towards_heading(heading=-135, speed=-40, turn_aggression=1, distance_mm=500)
        turn_to_heading(heading=-90, turn_aggression=0.5)
        Motors.allWheels.set_stopping(COAST)  # Always good while running into things
        Motors.allWheels.set_velocity(-20, PERCENT)
        Motors.allWheels.spin(FORWARD)
        bprint("Backing up")
        roll_roller()
    bprint("Cleaning up...")
    Motors.intake.set_velocity(0, PERCENT)
    Motors.intake.stop()
    Motors.roller.set_velocity(0, PERCENT)
    Motors.roller.stop()
    Motors.flywheel.set_velocity(0, PERCENT)
    Motors.flywheel.stop()
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.stop()
    if Globals.STOPPING_MODE == Constants.BRAKE:
        Motors.allWheels.set_stopping(BRAKE)
    elif Globals.STOPPING_MODE == Constants.COAST:
        Motors.allWheels.set_stopping(COAST)
    elif Globals.STOPPING_MODE == Constants.HOLD:
        Motors.allWheels.set_stopping(HOLD)
    bprint("Auton: Exit")
    auton_log("-----------Autonomous Complete------------")
    if Globals.AUTONOMOUS_LOG:
        Globals.AUTONOMOUS_LOG.close()


def on_driver() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            move_with_offset(left_motor=Motors.left, right_motor=Motors.right)


# <editor-fold desc="Simple Button Handlers">
def start_stop_flywheel() -> None:
    """
    A custom controller binding for starting and stopping the flywheel
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.FLYWHEEL_ACTIVE = not Globals.FLYWHEEL_ACTIVE
    if Globals.FLYWHEEL_ACTIVE:
        Motors.flywheel.set_velocity(65, PERCENT)
        Motors.flywheel.spin(FORWARD)
    else:
        Motors.flywheel.stop()


def start_stop_roller() -> None:
    """
    A custom controller binding for starting and stopping the roller manually
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.ROLLER_ACTIVE = not Globals.ROLLER_ACTIVE
    if Globals.ROLLER_ACTIVE:
        Motors.roller.set_velocity(75, PERCENT)
        Motors.roller.spin(REVERSE)
    else:
        Motors.roller.stop()


def start_loader() -> None:
    """
    A custom controller binding for starting the smart loader
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.INTAKE_ACTIVE = True


def loading_handler():
    """
    This function controls the smart loader and works with stopping it when the disk is loaded
    """
    #  Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    Motors.intake.set_velocity(80, PERCENT)
    Sensors.optical_disk.set_light_power(100, PERCENT)
    Sensors.optical_disk.set_light(LedStateType.ON)
    while True:
        if not Globals.PAUSE_LOADING_THREAD:
            if Globals.INTAKE_ACTIVE and not Globals.DISK_READY:
                Motors.intake.set_velocity(80, PERCENT)
                while get_optical_color(Sensors.optical_disk) != Constants.YELLOW and Globals.INTAKE_ACTIVE:
                    Motors.intake.spin(FORWARD)
                if Globals.INTAKE_ACTIVE:
                    Motors.intake.spin_for(REVERSE, 30, DEGREES)
                    Motors.intake.stop()
                    Globals.DISK_READY = True
                    Globals.INTAKE_ACTIVE = False
            if Globals.INTAKE_ACTIVE and Globals.DISK_READY:
                Motors.intake.spin_for(FORWARD, 360, DEGREES)
                Globals.INTAKE_ACTIVE = False
                Globals.DISK_READY = False


def reset_loader():
    """
    Reset the state of the loader if it gets stuck
    """
    Globals.INTAKE_ACTIVE = False
    Globals.DISK_READY = False
    Globals.PAUSE_LOADING_THREAD = False
    Motors.intake.stop()


def unload() -> None:
    """
    A custom controller binding for quickly reversing the roller and then reloading the current disk
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.PAUSE_LOADING_THREAD = True
    Motors.intake.spin(REVERSE)
    # while Sensors.controller.buttonX.pressing():
    #     wait(5)
    # Use this and comment out the above loop if you intend for the unload button to reverse for a set amount of time
    sleep(500)
    Motors.intake.stop()
    Globals.DISK_READY = False
    Globals.INTAKE_ACTIVE = True
    Globals.PAUSE_LOADING_THREAD = False


Sensors.controller.buttonA.pressed(start_stop_flywheel)
Sensors.controller.buttonB.pressed(start_loader)
Sensors.controller.buttonX.pressed(unload)
Sensors.controller.buttonL1.pressed(start_stop_roller)
Sensors.controller.buttonR1.pressed(reset_loader)


# </editor-fold>


# <editor-fold desc="Competition State Handlers">
def autonomous_handler() -> None:
    """
    Coordinate when to run the autonomous function using the vex competition library to read the game state.
    """
    autonomous_thread = Thread(on_autonomous)
    Globals.COMPETITION_STATE = Constants.AUTONOMOUS
    while competition.is_autonomous() and competition.is_enabled():
        sleep(10)
    Globals.COMPETITION_STATE = Constants.DISABLED
    autonomous_thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function using the vex competition library to read the game state.
    """
    driver_thread = Thread(on_driver)
    Globals.COMPETITION_STATE = Constants.DRIVER
    while competition.is_driver_control() and competition.is_enabled():
        sleep(10)
    driver_thread.stop()
    Globals.COMPETITION_STATE = Constants.DISABLED


# Register the competition functions
competition = Competition(driver_handler, autonomous_handler)
# </editor-fold>

if __name__ == "__main__":
    if Sensors.controller.buttonX.pressing():
        button_hold_start_time = brain.timer.time(SECONDS)
        while Sensors.controller.buttonX.pressing() and brain.timer.time(SECONDS) - button_hold_start_time < 3:
            wait(5)
            cclear()
            cprint("[" + "#" * round(brain.timer.time(SECONDS) - button_hold_start_time) + " " * (
                (3 - round(brain.timer.time(SECONDS) - button_hold_start_time))) + "]")
        if Sensors.controller.buttonX.pressing() and brain.timer.time(SECONDS) - button_hold_start_time > 3:
            Globals.TEAM = Constants.RED
            Globals.STOPPING_MODE = Constants.COAST
            Globals.AUTONOMOUS_TASK = Constants.BOTH_ROLLERS
            Globals.CONTROL_MODE = Constants.TANK
        else:
            setup()
    else:
        setup()
    if Globals.STOPPING_MODE == Constants.BRAKE:
        Motors.allWheels.set_stopping(BRAKE)
    elif Globals.STOPPING_MODE == Constants.COAST:
        Motors.allWheels.set_stopping(COAST)
    elif Globals.STOPPING_MODE == Constants.HOLD:
        Motors.allWheels.set_stopping(HOLD)
    # Wait until all buttons are released
    while any((Sensors.controller.buttonLeft.pressing(),
               Sensors.controller.buttonRight.pressing(),
               Sensors.controller.buttonA.pressing(),
               Sensors.controller.buttonB.pressing())):
        wait(5)
    cclear()
    bclear()
    cprint("Calibrating Gyro...")
    # The following calibration sequence is supposed to minimize user error by detecting rapid inertial input changes and restarting calibration
    calibrate_start_time = brain.timer.time(MSEC)
    Sensors.inertial.calibrate()  # Start a calibration
    while brain.timer.time(MSEC) - calibrate_start_time < 1500:
        if any((
                abs(Sensors.inertial.gyro_rate(AxisType.XAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.YAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT)):
            Sensors.inertial.calibrate()  # Restart a calibration
            calibrate_start_time = brain.timer.time(MSEC)  # Reset the calibration start time
            cclear()
            bclear()
            cprint("Calibrating")
            cprint("Do not touch")
            bprint("Do not touch the robot during calibration")
            bprint("If you did not touch the robot, increase GLobals.CALIBRATION_RESET_DPS_LIMIT")
    cclear()
    Sensors.inertial.set_heading(0, DEGREES)
    cprint("Setup complete")
    bprint("Setup complete")

    Thread(loading_handler)
    Globals.SETUP_COMPLETE = True
