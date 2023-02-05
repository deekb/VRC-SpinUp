"""
Competition Code for VRC: Spin-Up (2022-2023)
Author: Derek Baier (deekb on GithHub)
Project homepage: https://github.com/deekb/VexCode
Project archive https://github.com/deekb/VexCode/archive/master.zip
Version: 2.1.6_stable
Availible for use and modification under the creative commons liscense
Contact Derek.m.baier@gmail.com for more information
"""
# <editor-fold desc="Imports and liscense">
from vex import *
from Constants import Color, AutonomousTask
from HelperFunctions import BetterDrivetrain, cubic_normalize, controller_input_to_motor_power, move_with_offset, \
    get_optical_color, CustomPID, ToggleMotor

__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up 2022-2023"
__url__ = "https://github.com/deekb/VexCode"
__download_url__ = "https://github.com/deekb/VexCode/archive/master.zip"
__version__ = "2.1.6_stable"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "CC"
# </editor-fold>


brain = Brain()
brain.screen.set_font(FontType.MONO12)
brain.screen.draw_image_from_file('background.bmp', 0, 0)


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
    flywheel = CustomPID(Motor(Ports.PORT10, GearSetting.RATIO_36_1, True), kp=0.4, kd=0.05, t=0.01)
    intake = Motor(Ports.PORT13, GearSetting.RATIO_36_1, True)
    # Motor groups:
    leftDrivetrain = MotorGroup(leftFrontMotor, leftRearMotor)
    rightDrivetrain = MotorGroup(rightFrontMotor, rightRearMotor)
    allWheels = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)
    allMotors = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, roller, flywheel, intake)
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


class Globals:
    """
    Stores variable that may need to be (or ought to be able to be) accessed by any function in the program, here you can set default/initial values for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc,
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM = 125
    AUTONOMOUS_TASK = AutonomousTask.LEFT_ROLLER
    HEADING_OFFSET_TOLERANCE = 1  # How many degrees off is "Close enough"
    CALIBRATION_RESET_DPS_LIMIT = 5  # How many degrees per second does the inertial sensor have to report to invalidate and restart calibration
    TEAM = None
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    STOPPING_MODE = COAST
    FLYWHEEL_ACTIVE = False
    ROLLER_ACTIVE = False
    INTAKE_ACTIVE = False
    DISK_READY = False
    PAUSE_LOADING_THREAD = False
    SETTINGS = (
        ("Team", [("Red", Color.RED),
                  ("Blue", Color.BLUE)]),
        ("Stopping", [("Coast", COAST),
                      ("Brake", BRAKE)]),
        ("Autonomous", [("Shoot", AutonomousTask.SHOOT_PRELOAD),
                        ("Plow disks", AutonomousTask.PUSH_IN_DISKS_WITH_PLOW),
                        ("Roller", AutonomousTask.LEFT_ROLLER),
                        ("Both rollers", AutonomousTask.BOTH_ROLLERS),
                        ("Spit disks", AutonomousTask.SPIT_OUT_DISKS_WITH_INTAKE)])
    )


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


def setup() -> None:
    """
    A setup function, sets the globals to the values selected on the controller using its screen to print them
    """
    setting_index = 0
    while setting_index < len(Globals.SETTINGS):
        choice = 0
        setting_name = Globals.SETTINGS[setting_index][0]
        total_values = len(Globals.SETTINGS[setting_index][1])
        # Wait until all buttons are released
        while any((Sensors.controller.buttonLeft.pressing(),
                   Sensors.controller.buttonRight.pressing(),
                   Sensors.controller.buttonA.pressing(),
                   Sensors.controller.buttonB.pressing())):
            wait(5)
        while True:
            value_printable = Globals.SETTINGS[setting_index][1][choice][0]
            value = Globals.SETTINGS[setting_index][1][choice][1]
            cclear()
            cprint(setting_name + ": " + value_printable)
            if setting_index == 0:
                Globals.TEAM = value
            elif setting_index == 1:
                Globals.STOPPING_MODE = value
            elif setting_index == 2:
                Globals.AUTONOMOUS_TASK = value
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
            elif Sensors.controller.buttonB.pressing():
                setting_index -= 1
                break
            elif Sensors.controller.buttonA.pressing():
                setting_index += 1
                break
            # Wait until all buttons are released
            while any((Sensors.controller.buttonLeft.pressing(),
                       Sensors.controller.buttonRight.pressing(),
                       Sensors.controller.buttonA.pressing(),
                       Sensors.controller.buttonB.pressing())):
                wait(5)


def roll_roller(degrees=90):
    """
    Roll the roller 90 degrees
    :param degrees: Degrees to spin the roller
    :type degrees: float
    """
    Sensors.optical_roller.gesture_disable()
    Sensors.optical_roller.set_light_power(100, PERCENT)
    Sensors.optical_roller.set_light(LedStateType.ON)
    Motors.allWheels.set_stopping(COAST)  # Always good while running into things
    Motors.allWheels.set_velocity(-15, PERCENT)
    Motors.roller.set_velocity(80)
    Motors.allWheels.spin(FORWARD)
    while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
        wait(5)
    Motors.leftRearMotor.stop()
    Motors.rightRearMotor.stop()
    Motors.leftFrontMotor.set_velocity(-3, PERCENT)
    Motors.rightFrontMotor.set_velocity(-3, PERCENT)
    Motors.roller.spin_for(REVERSE, degrees * (20 / 9), DEGREES)
    Motors.allWheels.stop()
    wait(50)
    if get_optical_color(Sensors.optical_roller) != Globals.TEAM:
        roll_roller(degrees=10)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    if not Globals.SETUP_COMPLETE:
        return
    global drivetrain
    Motors.roller.set_stopping(HOLD)
    if Globals.AUTONOMOUS_TASK == AutonomousTask.PUSH_IN_DISKS_WITH_PLOW:
        # Autonomous to push a disk or stack of disks into the low goal with the plow
        drivetrain.move_towards_heading(heading=0, speed=20, distance_mm=500)
        drivetrain.move_towards_heading(heading=0, speed=-20, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SPIT_OUT_DISKS_WITH_INTAKE:
        # Autonomous to push a disk or stack of disks into the low goal with the intake
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(REVERSE)
        drivetrain.move_towards_heading(heading=0, speed=-20, distance_mm=500)
        drivetrain.move_towards_heading(heading=0, speed=20, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SHOOT_PRELOAD:
        # Autonomous to shoot a preload into the high goal from the center of the field
        Sensors.inertial.set_heading(0, DEGREES)
        drivetrain.turn_to_heading(heading=25)
        drivetrain.turn_to_heading(heading=45)
        drivetrain.move_towards_heading(heading=45, speed=50, distance_mm=1525)
        drivetrain.turn_to_heading(heading=-35)
        Motors.flywheel.set_velocity(80, PERCENT)
        Motors.flywheel.spin(FORWARD)
        while Motors.flywheel.velocity(PERCENT) < 60:
            wait(10)
        wait(3500)
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(FORWARD)
        wait(750)
        Motors.intake.spin(REVERSE)
        wait(400)
        Motors.intake.spin(FORWARD)
        wait(2000)
        Motors.intake.stop()
        Motors.flywheel.stop()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.LEFT_ROLLER:
        # Autonomous to roll a single roller without the optical sensor
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.BOTH_ROLLERS:
        # Autonomous to roll both rollers without the optical sensor
        roll_roller()
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.turn_to_heading(heading=0)
        drivetrain.move_towards_heading(heading=0, speed=40, distance_mm=100)
        drivetrain.turn_to_heading(heading=35)
        drivetrain.move_towards_heading(heading=35, speed=50, distance_mm=1075)
        drivetrain.turn_to_heading(heading=45)
        drivetrain.move_towards_heading(heading=45, speed=70, distance_mm=1700)
        drivetrain.turn_to_heading(heading=90)
        drivetrain.move_towards_heading(heading=90, speed=40, distance_mm=140)
        drivetrain.turn_to_heading(heading=-135)
        drivetrain.move_towards_heading(heading=-135, speed=-40, distance_mm=500)
        drivetrain.turn_to_heading(heading=-90)
        Motors.allWheels.set_stopping(COAST)  # Always good while running into things
        Motors.allWheels.set_velocity(-20, PERCENT)
        Motors.allWheels.spin(FORWARD)
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SKILLS:
        # Autonomous to roll all four rollers without the optical sensor
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(FORWARD)
        Motors.flywheel.set_velocity(25, PERCENT)
        Motors.flywheel.spin(FORWARD)
        drivetrain.turn_to_heading(heading=0)
        roll_roller(degrees=180)
        drivetrain.turn_to_heading(heading=0)
        drivetrain.move_towards_heading(heading=0, speed=40, distance_mm=700)
    bprint("Cleaning up...")
    Motors.intake.set_velocity(0, PERCENT)
    Motors.intake.stop()
    Motors.roller.set_velocity(0, PERCENT)
    Motors.roller.stop()
    Motors.flywheel.set_velocity(0, PERCENT)
    Motors.flywheel.stop()
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    bprint("Auton: Exit")
    if Globals.AUTONOMOUS_LOG:
        Globals.AUTONOMOUS_LOG.close()


def on_driver() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    global drivetrain
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            drivetrain.move_with_controller()


# <editor-fold desc="Simple Button Handlers">

ToggleMotor(motor=Motors.flywheel, speeds=[0, 65], event=Sensors.controller.buttonA.pressed)
"""def start_stop_flywheel() -> None:
    \"\"\"
    A custom controller binding for starting and stopping the flywheel
    \"\"\"
    if not Globals.SETUP_COMPLETE:
        return
    Globals.FLYWHEEL_ACTIVE = not Globals.FLYWHEEL_ACTIVE
    if Globals.FLYWHEEL_ACTIVE:
        Motors.flywheel.set_velocity(65, PERCENT)
        Motors.flywheel.spin(FORWARD)
    else:
        Motors.flywheel.stop()"""

ToggleMotor(motor=Motors.roller, speeds=[0, 75], event=Sensors.controller.buttonB.pressed)
"""def start_stop_roller() -> None:
    \"\"\"
    A custom controller binding for starting and stopping the roller manually
    \"\"\"
    if not Globals.SETUP_COMPLETE:
        return
    Globals.ROLLER_ACTIVE = not Globals.ROLLER_ACTIVE
    if Globals.ROLLER_ACTIVE:
        Motors.roller.set_velocity(75, PERCENT)
        Motors.roller.spin(REVERSE)
    else:
        Motors.roller.stop()"""


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
                while get_optical_color(Sensors.optical_disk) != Color.YELLOW and Color.INTAKE_ACTIVE:
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


# Sensors.controller.buttonA.pressed(start_stop_flywheel)
# Sensors.controller.buttonB.pressed(start_loader)
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
    while competition.is_autonomous() and competition.is_enabled():
        sleep(10)
    autonomous_thread.stop()


def driver_handler() -> None:
    """
    Coordinate when to run the driver function using the vex competition library to read the game state.
    """
    driver_thread = Thread(on_driver)
    while competition.is_driver_control() and competition.is_enabled():
        sleep(10)
    driver_thread.stop()


# Register the competition functions
competition = Competition(driver_handler, autonomous_handler)
# </editor-fold>

if __name__ == "__main__":
    if Sensors.controller.buttonX.pressing():
        cclear()
        cprint("Skipped setup")
        Globals.TEAM = Color.RED
        Globals.STOPPING_MODE = COAST
        Globals.AUTONOMOUS_TASK = AutonomousTask.BOTH_ROLLERS
    else:
        setup()
    # Apply the effect of the variable set during setup
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    drivetrain = BetterDrivetrain(inertial=Sensors.inertial, left_side=Motors.leftDrivetrain, right_side=Motors.rightDrivetrain, heading_offset_tolerance=Globals.HEADING_OFFSET_TOLERANCE, turn_aggression=0.5, correction_aggression=1, wheel_radius_mm=50)
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
