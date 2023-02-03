"""
Competition Code for VRC: Spin-Up (2022-2023)
Author: Derek Baier (deekb on GithHub)
Project homepage: https://github.com/deekb/VRC-SpinUp
Project archive https://github.com/deekb/VRC-SpinUp/archive/master.zip
Version: 6.5.0
Availible for use and modification without restriction
Contact Derek.m.baier@gmail.com for more information
"""
import os

# <editor-fold desc="Imports and liscense">

from vex import *

from VEX_UTILS import initial_code, bprint, bclear, cprint, cclear, CustomPID, \
    move_with_offset, move_towards_heading, turn_to_heading, auton_log, get_optical_color, roll_roller

exec(initial_code)  # This allows us to eobed code into the initial_code variable on the SD card and have it executed.
__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up 2022-2023"
__url__ = "https://github.com/deekb/VexCode"
__download_url__ = "https://github.com/deekb/VexCode/archive/master.zip"
__version__ = "3.2.0"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "CC"
# </editor-fold>


brain = Brain()


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
    ROLL_AND_SHOOT = 32
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
    AUTONOMOUS_TASK = Constants.ROLL_ROLLER
    HEADING_OFFSET_TOLERANCE = 1  # How many degrees off is "Close enough"
    CALIBRATION_RESET_DPS_LIMIT = 5  # How many degrees per second does the inertial sensor have to report to invalidate and restart calibration
    DELTA_HEADING_SAMPLE_RATE_MS = 50  # Take a delta_heading sample every X milliseconds, or None to disable delta_heading sampling
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


def setup() -> None:
    """
    A setup function, sets the global settings to the values selected on the controller using its screen to print them
    left and right keys to switch between then, "A" to accept and "B" to go back
    """
    settings = (
        ("Team", [("Red", Constants.RED),
                  ("Blue", Constants.BLUE)]),
        ("Stopping", [("Coast", Constants.COAST),
                      ("Brake", Constants.BRAKE)]),
        ("Auton", [("Shoot", Constants.SHOOT_PRELOAD),
                   ("RollShoot", Constants.ROLL_AND_SHOOT),
                   ("Plow", Constants.PUSH_IN_DISKS_WITH_PLOW),
                   ("Roller", Constants.ROLL_ROLLER),
                   ("2Roller", Constants.BOTH_ROLLERS),
                   ("Spit", Constants.SPIT_OUT_DISKS_WITH_INTAKE)]),
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
            setting_index -= 1  # Previous setting
        else:
            setting_index += 1  # Bext setting
        # Wait untill all buttons are released
        while any((Sensors.controller.buttonLeft.pressing(),
                   Sensors.controller.buttonRight.pressing(),
                   Sensors.controller.buttonA.pressing(),
                   Sensors.controller.buttonB.pressing())):
            wait(5)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    integer_names = [0]
    try:
        current_logs = os.listdir("Logs/Autonomous/")
        for log_name in current_logs:
            try:
                assert os.path.isfile(os.path.join("Logs/Autonomous/", log_name))
                log_name_int = int(log_name.replace(".log", ""))
            except (TypeError, ValueError, AssertionError):
                continue  # Skip this value as it is not an integer
            integer_names.append(log_name_int)
        Globals.AUTONOMOUS_LOG = open(os.path.join("Logs/Autonomous/" + str(max(integer_names) + 1) + ".log"), "a")  # "a" for append
    except (OSError, AttributeError, AssertionError):  # No SD card present
        Globals.AUTONOMOUS_LOG = None
    bprint("Autonomous:STATUS:Started")
    if auton_log("----------Autonomous log Open----------"):
        bprint("Autonomous:STATUS:Log opened")
    else:
        bprint("Autonomous:WARNING:Loging failed, SD card initialization error, ")
    if Globals.AUTONOMOUS_TASK == Constants.PUSH_IN_DISKS_WITH_PLOW:
        auton_log("Autonomous:DESCRIPTION: Autonomous will attempt to push a disk or stack of disks into the low goal with the plow")
        auton_log("Moving forward")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
        auton_log("Moving backward")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == Constants.SPIT_OUT_DISKS_WITH_INTAKE:
        auton_log("Autonomous:DESCRIPTION: Autonomous will attempt to push a disk or stack of disks into the low goal with the intake")
        Motors.intake.set_velocity(100, PERCENT)
        auton_log("Reversing loader")
        Motors.intake.spin(REVERSE)
        auton_log("Moving backward 500 mm")
        move_towards_heading(heading=0, speed=-20, turn_aggression=1, distance_mm=500)
        wait(1000)
        auton_log("Moving forward 500 mm")
        move_towards_heading(heading=0, speed=20, turn_aggression=1, distance_mm=500)
        Motors.intake.stop()
    elif Globals.AUTONOMOUS_TASK == Constants.SHOOT_PRELOAD:
        auton_log("Autonomous:DESCRIPTION: Autonomous will attempt to shoot a preload into the high goal from the center of the field")
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
        Motors.intake.set_velocity(80, PERCENT)
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
        auton_log("Autonomous:DESCRIPTION: Autonomous will attempt to roll the roller without the optical sensor")
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == Constants.BOTH_ROLLERS:
        auton_log("Autonomous will attempt to roll the roller without the optical sensor")
        roll_roller()
        Motors.allWheels.set_stopping(BRAKE)
        auton_log("Moving forward")
        turn_to_heading(heading=0, turn_aggression=0.5)
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
    elif Globals.AUTONOMOUS_TASK == Constants.ROLL_AND_SHOOT:
        auton_log("Autonomous will attempt to roll the roller and then shoot a preload into the high goal from the center of the field")
        roll_roller()
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
        Motors.intake.set_velocity(80, PERCENT)
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
    auton_log("Cleaning up...")
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
    auton_log("Autonomous: Clean Exit")
    bprint("Autonomous: Clean Exit")
    auton_log("-----------Autonomous Complete------------")
    if Globals.AUTONOMOUS_LOG:
        Globals.AUTONOMOUS_LOG.close()


def on_driver() -> None:
    """
    This is the function designated to run when the driver portion of the program is triggered
    """
    # Wait for setup to be complete
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    bprint("Driver: Cleaning up in case of unclean autonomous exit")
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
    auton_log("Driver: Done")
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            move_with_offset(left_motor=Motors.left, right_motor=Motors.right)  # Run a movement tick to take input from the remote and move the wheels


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
