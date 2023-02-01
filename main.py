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
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID
    :param kd: Kd value for the PID
    :param t: Time between PID updates
    """

    def __init__(self, motor_object, kp: float = 0.7, kd: float = 0.15, t: float = 0.01):
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
        Update the PID state with the most recent motor and target velocities and write the normalized value to the motor
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
        :param unit:  The velocity unit - only PERCENT is currently supported
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
    loader = Motor(Ports.PORT13, GearSetting.RATIO_36_1, True)
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
    optical_disk = Optical(Ports)
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
    ORANGE = 8
    YELLOW = 9
    GREEN = 10
    BLUE = 11
    PURPLE = 12
    ROLLER = 13
    SHOOTER = 14
    TANK = 15
    ARCADE = 16
    COAST = 17
    BRAKE = 18
    HOLD = 19
    YES = 20
    NO = 21
    ON = 22
    OFF = 23
    AUTONOMOUS = 24
    DRIVER = 25
    ENABLED = 26
    DISABLED = 27
    ROLL_ROLLER_SMART = 28
    PUSH_IN_DISKS_WITH_PLOW = 29
    SPIT_OUT_DISKS_WITH_INTAKE = 30
    SHOOT_PRELOAD = 31
    ROLL_ROLLER = 32
    BOTH_ROLLERS = 33
    PI = 3.141592


class Globals:
    """
    Stores variable that may need to be accessed by any cunction in the program, here you can set default/initial values
    for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc,
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    WHEEL_RADIUS_MM = 50
    WHEEL_CIRCUMFERENCE_MM = 2 * WHEEL_RADIUS_MM * Constants.PI
    ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM = 125
    AUTONOMOUS_TASK = 0
    DEBOUNCE_STABALIZE_SAMPLE_COUNT = 3
    HEADING_OFFSET_TOLERANCE = 1
    CALIBRATION_RESET_DPS_LIMIT = 5
    CONTROL_MODE = None
    COMPETITION_STATE = None
    TEAM = None
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    STOPPING_MODE = None
    FLYWHEEL_ACTIVE = False
    ROLLER_ACTIVE = False
    LOADER_ACTIVE = False
    STRING_UNLOAD_ACTIVE = False
    PREVIOUS_OPTICAL_VALUES = [None for _ in range(DEBOUNCE_STABALIZE_SAMPLE_COUNT)]
    AUTONOMOUS_LOG = None
    try:
        AUTONOMOUS_LOG = open("autonomous.log", "a")
    except (OSError, AttributeError):
        pass


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
        ("Control Mode", [("Tank", Constants.TANK),
                          ("Arcade", Constants.ARCADE)])
    )

    setting_index = 0
    while setting_index < len(settings):
        choice = 0
        setting_name = settings[setting_index][0]
        total_values = len(settings[setting_index][1])
        while Sensors.controller.buttonA.pressing():
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
            while any((Sensors.controller.buttonLeft.pressing(),
                       Sensors.controller.buttonRight.pressing(),
                       Sensors.controller.buttonA.pressing())):
                pass
            while not any((Sensors.controller.buttonLeft.pressing(),
                           Sensors.controller.buttonRight.pressing(),
                           Sensors.controller.buttonA.pressing())):
                pass
            if Sensors.controller.buttonRight.pressing() and choice < total_values - 1:
                choice += 1
            elif Sensors.controller.buttonLeft.pressing() and choice > 0:
                choice -= 1
        if Sensors.controller.buttonB.pressing():
            setting_index -= 1
        else:
            setting_index += 1
    if Globals.STOPPING_MODE == Constants.BRAKE:
        Motors.allWheels.set_stopping(BRAKE)
    elif Globals.STOPPING_MODE == Constants.COAST:
        Motors.allWheels.set_stopping(COAST)
    elif Globals.STOPPING_MODE == Constants.HOLD:
        Motors.allWheels.set_stopping(HOLD)
    # Wait until all buttons are released
    while any((Sensors.controller.buttonLeft.pressing(),
               Sensors.controller.buttonRight.pressing(),
               Sensors.controller.buttonA.pressing())):
        wait(5)
    cclear()
    bclear()
    cprint("Calibrating Gyro...")
    calibrate_start_time = brain.timer.time(MSEC)
    Sensors.inertial.calibrate()
    while brain.timer.time(MSEC) - calibrate_start_time < 1500:
        if any((
                abs(Sensors.inertial.gyro_rate(AxisType.XAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.YAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT)):
            Sensors.inertial.calibrate()
            calibrate_start_time = brain.timer.time(MSEC)
            cclear()
            bclear()
            cprint("Calibrating")
            cprint("Do not touch")
            bprint("Do not touch the robot during calibration")
            bprint("If you did not touch the robot, increase GLobals.CALIBRATION_RESET_DPS_LIMIT")
    cclear()
    Sensors.inertial.set_heading(0, DEGREES)
    cprint("Calibrated")
    bprint("Calibrated")
    cprint("Setup complete")
    bprint("Setup complete")
    Globals.SETUP_COMPLETE = True


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
    Motors.allWheels.stop()
    wait(10)
    current_heading = Sensors.inertial.heading(DEGREES) % 360
    auton_log("Turned to heading " + str(heading) + " with accuracy of " + str(abs(current_heading - heading)) + "  degrees")


def move_towards_heading(heading, speed, turn_aggression, distance_mm) -> None:
    """
    Move towards a heading using dynamic course correction
    :param heading: The absolute heading to move towards
    :param speed:  The base speed to move at
    :param turn_aggression: The multiplier for delta_heading
    :param distance_mm: The distance to move before stopping the movement
    """
    heading = heading % 360
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.spin(FORWARD)
    initial_distance_traveled = (((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                                 / 360) * Globals.WHEEL_CIRCUMFERENCE_MM
    distance_traveled = abs((((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                             / 360) * Globals.WHEEL_CIRCUMFERENCE_MM - initial_distance_traveled)
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

        if left_turn_difference < right_turn_difference:
            delta_heading = left_turn_difference
            Motors.left.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
            Motors.right.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
        else:
            delta_heading = right_turn_difference
            Motors.left.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
            Motors.right.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
    Motors.allWheels.stop()
    wait(10)
    distance_traveled = abs((((Motors.left.position(DEGREES) + Motors.right.position(DEGREES)) / 2)
                             / 360) * Globals.WHEEL_CIRCUMFERENCE_MM - initial_distance_traveled)
    current_heading = Sensors.inertial.heading(DEGREES) % 360
    auton_log("Moved towards heading " + str(heading) + " with end accuracy of " + str(abs(current_heading - heading))
              + "  degrees and distance accuracy of " + str(distance_traveled - distance_mm) + " mm")


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    bprint("Autonomous thread started")
    if auton_log("----------Autonomous log started----------"):
        bprint("Log opened")
    else:
        bprint("Loging failed, SD card error")
    if Globals.AUTONOMOUS_TASK == Constants.PUSH_IN_DISKS_WITH_PLOW:
        auton_log("Autonomous will attempt to push a disk or stack of disks into the low goal with the plow")
        auton_log("Moving forward")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
        auton_log("Moving backward")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == Constants.SPIT_OUT_DISKS_WITH_INTAKE:
        auton_log("Autonomous will attempt to push a disk or stack of disks into the low goal with the intake")
        Motors.loader.set_velocity(100, PERCENT)
        auton_log("Reversing loader")
        Motors.loader.spin(REVERSE)
        auton_log("Moving backward")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
        auton_log("Moving forward")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == Constants.SHOOT_PRELOAD:
        auton_log("Autonomous will attempt to shoot a preload into the high goal from the center of the field")
        auton_log("Reseting inertial sensor to allow correct orientation detection...")
        Sensors.inertial.set_heading(0, DEGREES)  # Reset the sensor
        auton_log("Inertial sensor reset complete")
        auton_log("Turning towards the center of field...")
        turn_to_heading(45, 0.5)
        auton_log("Done")
        auton_log("Moving towards the center of the field...")
        move_towards_heading(heading=45, speed=50, turn_aggression=1, distance_mm=1525)
        auton_log("Done")
        auton_log("Aiming at goal...")
        turn_to_heading(-38, 0.5)
        auton_log("Done")
        auton_log("Starting up flywheel")
        Motors.flywheel.set_velocity(75, PERCENT)
        Motors.flywheel.spin(FORWARD)
        auton_log("Waiting for full speed...")
        while Motors.flywheel.velocity(PERCENT) < 60:
            wait(10)
        auton_log("Done")
        auton_log("Waiting an aditional 3500 MS...")
        wait(3500)
        auton_log("Done")
        Motors.loader.set_velocity(100, PERCENT)
        auton_log("Firing")
        Motors.loader.spin(FORWARD)
        wait(750)
        auton_log("Unsticking disk")
        Motors.loader.spin(REVERSE)
        wait(400)
        auton_log("Firing")
        Motors.loader.spin(FORWARD)
        wait(2000)
        auton_log("Spinning down")
        Motors.loader.stop()
        Motors.flywheel.stop()
    elif Globals.AUTONOMOUS_TASK == Constants.ROLL_ROLLER:
        auton_log("Autonomous will attempt to roll the roller without the optical sensor")
        auton_log("Aligning")
        Motors.allWheels.set_stopping(COAST)  # Always good while running into things
        Motors.allWheels.set_velocity(-15, PERCENT)
        Motors.allWheels.spin(FORWARD)
        bprint("Backing up")
        while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
            cclear()
            cprint(Sensors.ultrasonic.distance(MM))
        Motors.leftFrontMotor.set_velocity(-1, PERCENT)
        Motors.rightFrontMotor.set_velocity(-1, PERCENT)
        bprint("Rolling roller")
        auton_log("Rolling roller")
        Motors.roller.set_velocity(80)
        Motors.roller.spin_for(REVERSE, 200, DEGREES)
        auton_log("Reseting stopping mode")
        if Globals.STOPPING_MODE == Constants.BRAKE:
            Motors.allWheels.set_stopping(BRAKE)
        elif Globals.STOPPING_MODE == Constants.COAST:
            Motors.allWheels.set_stopping(COAST)
        elif Globals.STOPPING_MODE == Constants.HOLD:
            Motors.allWheels.set_stopping(HOLD)
        Motors.allWheels.stop()
    elif Globals.AUTONOMOUS_TASK == Constants.BOTH_ROLLERS:
        auton_log("Autonomous will attempt to roll the roller without the optical sensor")
        auton_log("Aligning")
        Motors.allWheels.set_stopping(COAST)  # Always good while running into things
        Motors.allWheels.set_velocity(-15, PERCENT)
        Motors.allWheels.spin(FORWARD)
        bprint("Backing up")
        while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
            wait(5)
        Motors.leftFrontMotor.set_velocity(-1, PERCENT)
        Motors.rightFrontMotor.set_velocity(-1, PERCENT)
        bprint("Rolling roller")
        auton_log("Rolling roller")
        Motors.roller.set_velocity(80)
        Motors.roller.spin_for(REVERSE, 200, DEGREES)
        Motors.allWheels.set_stopping(BRAKE)
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=100)
        turn_to_heading(heading=45, turn_aggression=0.5)
        move_towards_heading(heading=25, speed=40, turn_aggression=1, distance_mm=500)
        move_towards_heading(heading=45, speed=40, turn_aggression=1, distance_mm=2500)
        turn_to_heading(heading=180, turn_aggression=0.5)
        move_towards_heading(heading=180, speed=-20, turn_aggression=1, distance_mm=300)
        turn_to_heading(heading=-90, turn_aggression=0.5)
        Motors.allWheels.set_stopping(COAST)  # Always good while running into things
        Motors.allWheels.set_velocity(-15, PERCENT)
        Motors.allWheels.spin(FORWARD)
        bprint("Backing up")
        while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
            wait(5)
        Motors.leftFrontMotor.set_velocity(-1, PERCENT)
        Motors.rightFrontMotor.set_velocity(-1, PERCENT)
        bprint("Rolling roller")
        auton_log("Rolling roller")
        Motors.roller.set_velocity(80)
        Motors.roller.spin_for(REVERSE, 200, DEGREES)
        if Globals.STOPPING_MODE == Constants.BRAKE:
            Motors.allWheels.set_stopping(BRAKE)
        elif Globals.STOPPING_MODE == Constants.COAST:
            Motors.allWheels.set_stopping(COAST)
        elif Globals.STOPPING_MODE == Constants.HOLD:
            Motors.allWheels.set_stopping(HOLD)
        Motors.allWheels.stop()
    auton_log("-----------Autonomous Complete------------")
    bprint("Done Auton")
    if Globals.AUTONOMOUS_LOG:
        Globals.AUTONOMOUS_LOG.close()


def on_driver() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    # Explicitly set motor states in case of an unclean autonomous termination
    Globals.PAUSE_DRIVER_CONTROL = False
    Motors.loader.set_velocity(0, PERCENT)
    Motors.loader.stop()
    Motors.roller.set_velocity(0, PERCENT)
    Motors.roller.stop()
    Motors.flywheel.set_velocity(0, PERCENT)
    Motors.flywheel.stop()
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            move_with_offset(left_motor=Motors.left, right_motor=Motors.right)


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


def start_stop_loader() -> None:
    """
    A custom controller binding for starting and stopping the loader
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.LOADER_ACTIVE = not Globals.LOADER_ACTIVE
    if Globals.LOADER_ACTIVE:
        Motors.loader.set_velocity(100, PERCENT)
        Motors.loader.spin(FORWARD)
    else:
        Motors.loader.stop()


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


def unload() -> None:
    """
    A custom controller binding for quickly reversing the roller and then returning it to its previous state
    """
    if not Globals.SETUP_COMPLETE:
        return
    Motors.loader.spin(REVERSE)
    sleep(500)
    if Globals.LOADER_ACTIVE:
        Motors.loader.spin(FORWARD)
    else:
        Motors.loader.stop()


Sensors.controller.buttonA.pressed(start_stop_flywheel)
Sensors.controller.buttonB.pressed(start_stop_loader)
Sensors.controller.buttonX.pressed(unload)
Sensors.controller.buttonL1.pressed(start_stop_roller)
# </editor-fold>

if __name__ == "__main__":
    setup()
