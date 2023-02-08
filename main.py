"""
Competition Code for VRC: Spin-Up (2022-2023)
Team: 3773P (Bowbots Phosphorus)
Author: Derek Baier (deekb on GithHub)
Project homepage: https://github.com/deekb/VRC-SpinUp
Project archive: https://github.com/deekb/VRC-SpinUp/archive/master.zip
Version: 2.4.18_stable
Liscense: If you are using or modifying this for your own robot there is no need to give credit unless you think it neccesary
Contact Derek.m.baier@gmail.com for more information
"""
# <editor-fold desc="Imports and liscense">
from vex import *
from Constants import Color, AutonomousTask
from HelperFunctions import BetterDrivetrain, cubic_normalize, controller_input_to_motor_power, move_with_controller, \
    get_optical_color, CustomPID

__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up 2022-2023"
__team__ = "3773P (Bowbots Phosphorus)"
__url__ = "https://github.com/deekb/VRC-SpinUp"
__download_url__ = "https://github.com/deekb/VRC-SpinUp/archive/master.zip"
__version__ = "2.4.18_stable"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "If you are using or modifying this for your own robot there is no need to give credit unless you think it neccesary"
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
    expansion = Motor(Prrts.PORT, GearSetting.RATIO_36_1, False)
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
    optical_roller = Optical(Ports.PORT14)
    optical_disk = Optical(Ports.PORT8)
    ultrasonic = Sonar(brain.three_wire_port.g)


class Controllers:
    """
    Primary and secondary controllers
    """
    primary = Controller(PRIMARY)
    secondary = Controller(PARTNER)


class Globals:
    """
    Stores variable that may need to be (or ought to be able to be) accessed by any function in the program, here you can set default/initial values for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc,
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM = 125
    AUTONOMOUS_TASK = AutonomousTask.ROLLER_LEFT
    HEADING_OFFSET_TOLERANCE = 1  # How many degrees off is "Close enough"
    CALIBRATION_RESET_DPS_LIMIT = 5  # How many degrees per second does the inertial sensor have to report to invalidate and restart calibration
    EXPANSION_REMINDER_TIME_MSEC = 95000  # The amount of time after driver control starts that the program should vibrate the secondary controller
    TEAM = None
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    STOPPING_MODE = COAST
    FLYWHEEL_ACTIVE = False
    ROLLER_ACTIVE = False
    INTAKE_ACTIVE = False
    DISK_READY = False
    PAUSE_LOADING_THREAD = False
    DRIVER_START_TIME_MSEC = None
    SETTINGS = (
        ("Team", [("Red", Color.RED),
                  ("Blue", Color.BLUE)]),
        ("Stopping", [("Coast", COAST),
                      ("Brake", BRAKE),
                      ("Hold", HOLD)]),
        ("Autonomous", [("Both rollers", AutonomousTask.ROLLER_BOTH),
                        ("ShootL", AutonomousTask.SHOOT_PRELOAD_LEFT),
                        ("ShootR", AutonomousTask.SHOOT_PRELOAD_RIGHT),
                        ("RollerL", AutonomousTask.ROLLER_LEFT),
                        ("RollerR", AutonomousTask.ROLLER_RIGHT),
                        ("Plow disks", AutonomousTask.PUSH_IN_DISKS_WITH_PLOW),
                        ("Spit disks", AutonomousTask.SPIT_OUT_DISKS_WITH_INTAKE),
                        ("Low goal", AutonomousTask.SCORE_IN_LOW_GOAL),
                        ("Nothing", AutonomousTask.DO_NOTHING)])
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


def cprint(string, controller: Controller = Controllers.primary) -> None:
    """
    Print a string to the controller screen
    :param controller: The controller to print to
    :param string: the string to print to the screen
    """
    controller.screen.print(string)
    controller.screen.next_row()


def cclear(controller: Controller = Controllers.primary) -> None:
    """
    :param controller: The controller to clear
    Clears the controller screen
    """
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)


# </editor-fold>


bprint("Program: " + __title__)
bprint("Version: " + __version__)
bprint("Author: " + __author__)
bprint("Team: " + __team__)


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
        while any((Controllers.primary.buttonLeft.pressing(),
                   Controllers.primary.buttonRight.pressing(),
                   Controllers.primary.buttonA.pressing(),
                   Controllers.primary.buttonB.pressing())):
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
            while not any((Controllers.primary.buttonLeft.pressing(),
                           Controllers.primary.buttonRight.pressing(),
                           Controllers.primary.buttonA.pressing(),
                           Controllers.primary.buttonB.pressing())):
                wait(5)
            if Controllers.primary.buttonRight.pressing() and choice < total_values - 1:
                choice += 1
            elif Controllers.primary.buttonLeft.pressing() and choice > 0:
                choice -= 1
            elif Controllers.primary.buttonB.pressing():
                setting_index -= 1
                break
            elif Controllers.primary.buttonA.pressing():
                setting_index += 1
                break
            # Wait until all buttons are released
            while any((Controllers.primary.buttonLeft.pressing(),
                       Controllers.primary.buttonRight.pressing(),
                       Controllers.primary.buttonA.pressing(),
                       Controllers.primary.buttonB.pressing())):
                wait(5)


def roll_roller(degrees=90) -> None:
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
    Motors.leftFrontMotor.set_velocity(-3, PERCENT)
    Motors.rightFrontMotor.set_velocity(-3, PERCENT)
    Motors.roller.spin_for(REVERSE, degrees * (20 / 9), DEGREES)
    Motors.allWheels.stop()
    Motors.allWheels.set_stopping(BRAKE)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    if not Globals.SETUP_COMPLETE:
        return
    global drivetrain
    brain.screen.set_font(FontType.MONO12)
    Motors.roller.set_stopping(HOLD)
    Motors.allWheels.set_stopping(BRAKE)
    Sensors.inertial.set_heading(0, DEGREES)
    bprint("Autonomous: START")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DO_NOTHING:
        # Autonomous to well... do nothing!
        bprint("Autonomous:STATUS: Doing nothing")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DRIVETRAIN_TEST:
        # Test the drivetrain
        bprint("Autonomous:STATUS: Tesing drivetrain")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.SCORE_IN_LOW_GOAL:
        bprint("Autonomous:STATUS: Running score in low goal")
        raise NotImplementedError("Scoring in low goal not implemented")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.PUSH_IN_DISKS_WITH_PLOW:
        bprint("Autonomous:STATUS: Running plow in disks")
        # Autonomous to push a disk or stack of disks into the low goal with the plow
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=20, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-20, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SPIT_OUT_DISKS_WITH_INTAKE:
        bprint("Autonomous:STATUS: Running Spit out disks")
        # Autonomous to spit a disk or mutiple disks into the low goal with the intake after backing up
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-20, distance_mm=500)
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(REVERSE)
        wait(2500)
        Motors.intake.spin(FORWARD)
        wait(500)
        Motors.intake.spin(REVERSE)
        wait(2500)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=20, distance_mm=500)
        Motors.intake.stop()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SHOOT_PRELOAD_LEFT:
        bprint("Autonomous:STATUS: Running shoot preload (left)")
        # Autonomous to shoot a preload into the high goal from left position
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=50, distance_mm=1525)
        drivetrain.turn_to_heading(desired_heading=-35)
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
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SHOOT_PRELOAD_RIGHT:
        bprint("Autonomous:STATUS: Running shoot preload (right)")
        bprint("Autonomous:WARNING: Not tested")
        # Autonomous to shoot a preload into the high goal from the right position
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=50, distance_mm=1000)
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=50, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=35)
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
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_LEFT:
        # Autonomous to roll the left roller without the optical sensor
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_RIGHT:
        bprint("Autonomous:STATUS: Rolling right roller")
        # Autonomous to roll the right roller without the optical sensor
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-50, distance_mm=450)
        drivetrain.turn_to_heading(desired_heading=55)
        drivetrain.move_towards_heading(desired_heading=55, speed=-50, distance_mm=280)
        drivetrain.turn_to_heading(desired_heading=90)
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_BOTH:
        bprint("Autonomous:STATUS: Rolling both rollers")
        # Autonomous to roll both rollers without the optical sensor
        roll_roller()
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=40, distance_mm=100)
        drivetrain.turn_to_heading(desired_heading=35)
        drivetrain.move_towards_heading(desired_heading=35, speed=50, distance_mm=1075)
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=70, distance_mm=1700)
        drivetrain.turn_to_heading(desired_heading=90)
        Motors.intake.set_velocity(50, PERCENT)
        Motors.flywheel.set_velocity(20, PERCENT)
        Motors.intake.spin(FORWARD)
        Motors.flywheel.spin(FORWARD)
        drivetrain.move_towards_heading(desired_heading=90, speed=40, distance_mm=140)
        drivetrain.turn_to_heading(desired_heading=-135)
        drivetrain.move_towards_heading(desired_heading=-135, speed=-40, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=-90)
        roll_roller()
        Motors.intake.stop()
        Motors.flywheel.stop()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SKILLS:
        bprint("Autonomous:STATUS: Running skills")
        # Autonomous to roll all four rollers without the optical sensor
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(FORWARD)
        Motors.flywheel.set_velocity(25, PERCENT)
        Motors.flywheel.spin(FORWARD)
        drivetrain.turn_to_heading(desired_heading=0)
        roll_roller(degrees=180)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=40, distance_mm=650)
        drivetrain.turn_to_heading(desired_heading=90)
        drivetrain.move_towards_heading(desired_heading=90, speed=-40, distance_mm=650)
        roll_roller(degrees=180)
        drivetrain.turn_to_heading(desired_heading=90)
    bprint("Autonomous:INFO: Cleaning up")
    Motors.intake.set_velocity(0, PERCENT)
    Motors.intake.stop()
    Motors.roller.set_velocity(0, PERCENT)
    Motors.roller.stop()
    Motors.flywheel.set_velocity(0, PERCENT)
    Motors.flywheel.stop()
    Motors.allWheels.set_velocity(0, PERCENT)
    Motors.allWheels.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    bprint("Autonomous:STATUS: Exit")


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
            move_with_controller(left_side=Motors.leftDrivetrain, right_side=Motors.rightDrivetrain, controller=Controllers.primary, linearity=Globals.SPEED_CURVE_LINEARITY)


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


# </editor-fold>


def start_loader() -> None:
    """
    A custom controller binding for starting the smart loader
    """
    if Globals.SETUP_COMPLETE:
        Globals.INTAKE_ACTIVE = True


def loading_handler() -> None:
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
                while get_optical_color(Sensors.optical_disk) != Color.YELLOW and Globals.INTAKE_ACTIVE:
                    Motors.intake.spin(FORWARD)
                Motors.intake.stop()
                if Globals.INTAKE_ACTIVE:
                    Motors.intake.spin_for(REVERSE, 30, DEGREES)
                    Globals.DISK_READY = True
                    Globals.INTAKE_ACTIVE = False
            if Globals.INTAKE_ACTIVE and Globals.DISK_READY:
                Motors.intake.spin_for(FORWARD, 360, DEGREES)
                Globals.INTAKE_ACTIVE = False
                Globals.DISK_READY = False


def reminder_handler() -> None:
    """
    Remind the user at certain time intervals by buzzing their controller
    """
    while not (competition.is_enabled() and competition.is_driver_control()):
        wait(5)
    while True:
        if brain.timer.time(MSEC) - Globals.DRIVER_START_TIME_MSEC > Globals.EXPANSION_REMINDER_TIME_MSEC:
            Controllers.secondary.haptic_feddback("...")
            break


def reset_loader() -> None:
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
    sleep(500)
    Motors.intake.stop()
    Globals.DISK_READY = False
    Globals.INTAKE_ACTIVE = True
    Globals.PAUSE_LOADING_THREAD = False


def unload_no_reload() -> None:
    """
    A custom controller binding for quickly reversing the roller and then reloading the current disk
    """
    if not Globals.SETUP_COMPLETE:
        return
    Globals.PAUSE_LOADING_THREAD = True
    Motors.intake.spin(REVERSE)
    while Controllers.primary.buttonR2.pressing():
        wait(5)
    Motors.intake.stop()
    Globals.DISK_READY = False
    Globals.INTAKE_ACTIVE = False
    Globals.PAUSE_LOADING_THREAD = False


def fire_expansion() -> None:
    """
    Fire the expansion module
    """
    Motors.expansion.set_velocity(100, PERCENT)
    Motors.expansion.spin_for(FORWARD, 45, DEGREES)


# Primary controller bindings
Controllers.primary.buttonA.pressed(start_stop_flywheel)
Controllers.primary.buttonB.pressed(start_loader)
Controllers.primary.buttonX.pressed(unload)
Controllers.primary.buttonR2.pressed(unload_no_reload)
Controllers.primary.buttonL1.pressed(start_stop_roller)
Controllers.primary.buttonR1.pressed(reset_loader)
Controllers.primary.buttonL2.pressed(fire_expansion)

# Secondary controller bindings
Controllers.secondary.buttonA.pressed(start_stop_flywheel)
Controllers.secondary.buttonB.pressed(start_loader)
Controllers.secondary.buttonX.pressed(unload)
Controllers.secondary.buttonR2.pressed(unload_no_reload)
Controllers.secondary.buttonL1.pressed(start_stop_roller)
Controllers.secondary.buttonR1.pressed(reset_loader)
Controllers.secondary.buttonL2.pressed(fire_expansion)


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
    Globals.DRIVER_START_TIME_MSEC = brain.timer.time()
    while competition.is_driver_control() and competition.is_enabled():
        sleep(10)
    driver_thread.stop()


# Register the competition functions
competition = Competition(driver_handler, autonomous_handler)
# </editor-fold>

if __name__ == "__main__":
    brain.screen.set_font(FontType.MONO12)
    if Controllers.primary.buttonX.pressing():
        cclear()
        cprint("Skipped setup")
        Globals.TEAM = Color.RED
        Globals.STOPPING_MODE = COAST
        Globals.AUTONOMOUS_TASK = AutonomousTask.ROLLER_BOTH
    else:
        setup()
    # Apply the effect of seting Globals.STOPPING_MODE during setup
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    # Initialize a new smart drivetrain from our helper functions module (Not the vex one)
    drivetrain = BetterDrivetrain(inertial=Sensors.inertial, left_side=Motors.leftDrivetrain, right_side=Motors.rightDrivetrain, wheel_radius_mm=50, turn_aggression=0.5, correction_aggression=0.5, heading_offset_tolerance=1)
    cprint("Calibrating Gyro...")
    # The following calibration sequence is supposed to minimize user error by detecting rapid inertial input changes and restarting calibration
    Sensors.inertial.calibrate()  # Start a calibration
    while Sensors.inertial.is_calibrating():
        if any((
                abs(Sensors.inertial.gyro_rate(AxisType.XAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.YAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT,
                abs(Sensors.inertial.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS)) > Globals.CALIBRATION_RESET_DPS_LIMIT)):
            Sensors.inertial.calibrate()  # Restart a calibration
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
    Thread(reminder_handler)
    Thread(loading_handler)
    Globals.SETUP_COMPLETE = True
