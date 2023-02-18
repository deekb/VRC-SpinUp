"""
Competition Code for VRC: Spin-Up (2022-2023)
Team: 3773P (Bowbots Phosphorus)
Author: Derek Baier (deekb on GithHub)
Project homepage: https://github.com/deekb/VRC-SpinUp
Project archive: https://github.com/deekb/VRC-SpinUp/archive/master.zip
Version: 2.6.0_stable
Contact Derek.m.baier@gmail.com for more information
"""
# <editor-fold desc="Imports and liscense">
from vex import *
from Constants import Color, AutonomousTask
from HelperFunctions import BetterDrivetrain, cubic_normalize, controller_input_to_motor_power, \
    get_optical_color, CustomPID, Logging

__title__ = "Vex V5 2023 Competition code"
__description__ = "Competition Code for VRC: Spin-Up 2022-2023"
__team__ = "3773P (Bowbots Phosphorus)"
__url__ = "https://github.com/deekb/VRC-SpinUp"
__download_url__ = "https://github.com/deekb/VRC-SpinUp/archive/master.zip"
__version__ = "2.6.0_stable"
__author__ = "Derek Baier"
__author_email__ = "Derek.m.baier@gmail.com"
__license__ = "MIT"
# </editor-fold>


brain = Brain()
brain.screen.set_font(FontType.MONO12)
brain.screen.draw_image_from_file('background.bmp', 0, 0)


class Motors:
    """
    A class containing references to all motors and motor groups attatched to the robot including motors with custom PIDs
    """
    # Motors:
    leftFrontMotor = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
    leftRearMotor = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
    rightFrontMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
    rightRearMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
    roller = Motor(Ports.PORT12, GearSetting.RATIO_36_1, False)
    flywheel = CustomPID(Motor(Ports.PORT21, GearSetting.RATIO_36_1, True), kp=0.4, kd=0.05, t=0.01)
    intake = Motor(Ports.PORT14, GearSetting.RATIO_36_1, True)
    expansion = Motor(Ports.PORT13, GearSetting.RATIO_36_1, True)
    # Motor groups:
    leftDrivetrain = MotorGroup(leftFrontMotor, leftRearMotor)
    rightDrivetrain = MotorGroup(rightFrontMotor, rightRearMotor)
    allWheels = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor)
    allMotors = MotorGroup(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, roller, flywheel, intake)
    # PID handler threads:
    Thread(flywheel.PID_loop)


class Sensors:
    """
    A class that contains references to all sensors attatched to the robot
    """
    inertial = Inertial(Ports.PORT1)
    optical_disk = Optical(Ports.PORT10)
    ultrasonic = Sonar(brain.three_wire_port.g)


class Controllers:
    """
    A class that contains references to the primary and secondary controllers
    """
    primary = Controller(PRIMARY)
    secondary = Controller(PARTNER)


class Globals:
    """
    Stores variables that may need to be (or ought to be able to be) accessed by any function in the program, here you can also set default/initial values for said variables
    """
    # SPEED_CURVE_LINEARITY is demonstrated on this graph https://www.desmos.com/calculator/zoc7drp2pc
    # it should be set between 0.00 and 3.00 for optimal performance
    SPEED_CURVE_LINEARITY = 0.35
    ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM = 115  # The distance between the ultrasonic sensor and the wall when the flex wheel is touching the roller
    TEAM = None
    SETUP_COMPLETE = False
    PAUSE_DRIVER_CONTROL = False
    STOPPING_MODE = COAST
    FLYWHEEL_ACTIVE = False
    FLYWHEEL_STABLE = False
    FLYWHEEL_TARGET_POWER = 65
    FLYWHEEL_START_TIME_MSEC = None
    FLYWHEEL_SPINUP_TIME_MSEC = 1000  # If the flywheel reports spinup in < 1000 ms then wait the 1000 ms
    ROLLER_ACTIVE = False
    INTAKE_ACTIVE = False
    DISK_READY = False
    PAUSE_LOADING_THREAD = False
    DRIVER_START_TIME_MSEC = None
    FLYWHEEL_AUTOSTART = False
    AUTONOMOUS_TASK = None
    SETTINGS = (
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
                        ("Skills", AutonomousTask.SKILLS),
                        ("Test drivetrain", AutonomousTask.DRIVETRAIN_TEST),
                        ("Nothing", AutonomousTask.DO_NOTHING)]),
        ("Flywheel autostart", [("Yes", True), ("No", False)])
    )
    # All timers are in milliseconds from driver control start and will be canceled if the competition switches state during the timer
    # and re-started when it switches back. The timers trigger haptict on the secondary remote, every "." is a short rumble and every "-" is a long rumble
    TIMERS = {
        95000: "..."  # Reminder to expand
    }


# <editor-fold desc="Print/Clear functions">
def bprint(string) -> None:
    """
    Prints a string to the brain's screen
    :param string: the string to print to the screen
    """
    brain.screen.print(string)
    brain.screen.next_row()


def bclear() -> None:
    """
    Clears the brain's screen
    """
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)


def cprint(string, controller: Controller = Controllers.primary) -> None:
    """
    Prints a string to a controller's screen
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
                Globals.STOPPING_MODE = value
            elif setting_index == 1:
                Globals.AUTONOMOUS_TASK = value
            elif setting_index == 2:
                Globals.FLYWHEEL_AUTOSTART = value
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


def roll_roller(degrees=70) -> None:
    """
    Spin the roller
    :param degrees: Degrees to spin the roller
    :type degrees: float
    """
    Motors.allWheels.set_stopping(COAST)  # Always good while running into things
    Motors.roller.set_stopping(BRAKE)
    Motors.allWheels.set_velocity(-25, PERCENT)
    Motors.roller.set_velocity(30)
    Motors.allWheels.spin(FORWARD)
    while Sensors.ultrasonic.distance(MM) > Globals.ULTRASONIC_BACKUP_COMPLETE_DISTANCE_MM:
        wait(5)
    Motors.leftFrontMotor.set_velocity(-1, PERCENT)
    Motors.rightFrontMotor.set_velocity(-1, PERCENT)
    Motors.roller.spin_for(REVERSE, degrees, DEGREES)
    Motors.allWheels.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)


def on_autonomous() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    if not Globals.SETUP_COMPLETE:
        bprint("[on_autonomous]: setup not complete, ignoring request")
        return
    global drivetrain  # Ensure we can access the custom drivetrain
    autonomous_log = Logging(log_format="[%n]:%m:%s\n", mode="wt")

    def auton_log(string):
        """
        Send a string to the log and the brain screen
        :param string: The string to send
        """
        autonomous_log.log(string, "on_autonomous")
        bprint(string)

    brain.screen.set_font(FontType.MONO12)
    Motors.roller.set_stopping(HOLD)
    Motors.allWheels.set_stopping(BRAKE)
    drivetrain.reset()
    auton_log("Autonomous: START")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DO_NOTHING:
        # Autonomous to well... do nothing!
        auton_log("Autonomous:STATUS: Doing nothing")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.DRIVETRAIN_TEST:
        # Test the drivetrain
        auton_log("Autonomous:STATUS: Tesing drivetrain")
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=20, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=90)
        drivetrain.move_towards_heading(desired_heading=90, speed=30, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=180)
        drivetrain.move_towards_heading(desired_heading=180, speed=40, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=270)
        drivetrain.move_towards_heading(desired_heading=270, speed=50, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=20, distance_mm=1000)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-20, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=90)
        drivetrain.move_towards_heading(desired_heading=90, speed=-30, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-40, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=-90)
        drivetrain.move_towards_heading(desired_heading=-90, speed=-50, distance_mm=500)
        drivetrain.move_to_position(x=0, y=0, speed=20)
        auton_log("Autonomous:STATUS: Test complete")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.SCORE_IN_LOW_GOAL:
        auton_log("Autonomous:STATUS: Running score in low goal")
        raise NotImplementedError("Scoring in low goal not implemented")
    if Globals.AUTONOMOUS_TASK == AutonomousTask.PUSH_IN_DISKS_WITH_PLOW:
        auton_log("Autonomous:STATUS: Running plow in disks")
        # Autonomous to push a disk or stack of disks into the low goal with the plow
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=20, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-20, distance_mm=500)
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SPIT_OUT_DISKS_WITH_INTAKE:
        auton_log("Autonomous:STATUS: Running Spit out disks")
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
        auton_log("Autonomous:STATUS: Running shoot preload (left)")
        # Autonomous to shoot a preload into the high goal from left position
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=50, distance_mm=1525)
        drivetrain.turn_to_heading(desired_heading=-45)
        Motors.flywheel.set_velocity(95, PERCENT)
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
        auton_log("Autonomous:STATUS: Running shoot preload (right)")
        auton_log("Autonomous:WARNING: Not tested")
        # Autonomous to shoot a preload into the high goal from the right position
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=50, distance_mm=1000)
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=50, distance_mm=500)
        drivetrain.turn_to_heading(desired_heading=35)
        Motors.flywheel.set_velocity(80, PERCENT)
        Motors.flywheel.spin(FORWARD)
        Globals.FLYWHEEL_START_TIME_MSEC = brain.timer.time(MSEC)
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(FORWARD)
        while get_optical_color(optical_sensor=Sensors.optical_disk) != Color.YELLOW:
            pass
        Motors.intake.spin_for(REVERSE, 30, DEGREES)
        while Motors.flywheel.velocity(PERCENT) < 75 or brain.timer.time(MSEC) - Globals.FLYWHEEL_START_TIME_MSEC < Globals.FLYWHEEL_SPINUP_TIME_MSEC:
            pass
        wait(100)
        Motors.intake.spin_for(FORWARD, 360, DEGREES)
        Motors.intake.stop()
        Motors.flywheel.stop()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_LEFT:
        # Autonomous to roll the left roller without the optical sensor
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_RIGHT:
        auton_log("Autonomous:STATUS: Running roller (Right)")
        # Autonomous to roll the right roller without the optical sensor
        drivetrain.turn_to_heading(desired_heading=0)
        drivetrain.move_towards_heading(desired_heading=0, speed=-50, distance_mm=450)
        drivetrain.turn_to_heading(desired_heading=90)
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.ROLLER_BOTH:
        auton_log("Autonomous:STATUS: Running roller (Left)")
        # Autonomous to roll both rollers without the optical sensor
        drivetrain.turn_to_heading(desired_heading=0)
        roll_roller()
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.move_towards_heading(desired_heading=0, speed=60, distance_mm=100)
        drivetrain.turn_to_heading(desired_heading=35)
        drivetrain.move_towards_heading(desired_heading=35, speed=90, distance_mm=1275)
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=90, distance_mm=1600)
        drivetrain.turn_to_heading(desired_heading=90)
        drivetrain.move_towards_heading(desired_heading=90, speed=-70, distance_mm=140)
        Motors.intake.set_velocity(50, PERCENT)
        Motors.intake.spin(FORWARD)
        drivetrain.turn_to_heading(desired_heading=-135)
        drivetrain.move_towards_heading(desired_heading=-135, speed=-60, distance_mm=800)
        Motors.intake.stop()
        drivetrain.turn_to_heading(desired_heading=-90)
        roll_roller()
    elif Globals.AUTONOMOUS_TASK == AutonomousTask.SKILLS:
        auton_log("Autonomous:STATUS: Running skills")
        # Autonomous to roll all four rollers without the optical sensor
        Motors.intake.set_velocity(100, PERCENT)
        Motors.intake.spin(FORWARD)
        drivetrain.turn_to_heading(desired_heading=0)
        roll_roller(degrees=140)
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.move_towards_heading(desired_heading=0, speed=40, distance_mm=650)
        drivetrain.turn_to_heading(desired_heading=90)
        drivetrain.move_towards_heading(desired_heading=90, speed=-40, distance_mm=570)
        roll_roller(degrees=140)
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.move_towards_heading(desired_heading=90, speed=40, distance_mm=1200)
        drivetrain.turn_to_heading(desired_heading=45)
        drivetrain.move_towards_heading(desired_heading=45, speed=40, distance_mm=2500)
        drivetrain.turn_to_heading(desired_heading=-90)
        roll_roller(degrees=140)
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.move_towards_heading(desired_heading=-90, speed=40, distance_mm=650)
        drivetrain.turn_to_heading(desired_heading=-180)
        drivetrain.move_towards_heading(desired_heading=-180, speed=-40, distance_mm=650)
        roll_roller(degrees=140)
        Motors.allWheels.set_stopping(BRAKE)
        drivetrain.move_towards_heading(desired_heading=-180, speed=40, distance_mm=650)
        drivetrain.turn_to_heading(desired_heading=-135)
        drivetrain.move_towards_heading(desired_heading=-135, speed=-40, distance_mm=600)
        drivetrain.turn_to_heading(desired_heading=-135)
        fire_expansion()
        wait(1000)
        drivetrain.turn_to_heading(desired_heading=-142)
        drivetrain.move_towards_heading(desired_heading=-142, speed=-40, distance_mm=620)
        drivetrain.turn_to_heading(desired_heading=-90)
        drivetrain.move_towards_heading(desired_heading=-142, speed=-40, distance_mm=110)
    auton_log("Autonomous:INFO: Cleaning up")
    Motors.allMotors.stop()
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    auton_log("Autonomous:STATUS: Exit")
    autonomous_log.exit()


def on_driver() -> None:
    """
    This is the function designated to run when the autonomous portion of the program is triggered
    """
    # Wait for setup to be complete
    bprint("[on_driver]: Waiting for setup thread")
    while not Globals.SETUP_COMPLETE:
        sleep(5)
    bprint("[on_driver]: Done")
    global drivetrain
    Motors.allWheels.spin(FORWARD)
    while True:
        if not Globals.PAUSE_DRIVER_CONTROL:
            drivetrain.move_with_controller(Controllers.primary)


# <editor-fold desc="Simple Button Handlers">


def start_stop_flywheel() -> None:
    """
    A custom controller binding for starting and stopping the flywheel
    """
    if not Globals.SETUP_COMPLETE:
        bprint("[start_stop_flywheel]: setup not complete, ignoring request")
        return
    Globals.FLYWHEEL_ACTIVE = not Globals.FLYWHEEL_ACTIVE
    Globals.FLYWHEEL_STABLE = False
    if Globals.FLYWHEEL_ACTIVE:
        Motors.flywheel.set_velocity(Globals.FLYWHEEL_TARGET_POWER, PERCENT)
        Motors.flywheel.spin(FORWARD)
        Globals.FLYWHEEL_START_TIME_MSEC = brain.timer.time(MSEC)
    else:
        Motors.flywheel.stop()
        Globals.FLYWHEEL_STABLE = False


def start_stop_roller() -> None:
    """
    A custom controller binding for starting and stopping the roller manually
    """
    if not Globals.SETUP_COMPLETE:
        bprint("[start_stop_roller]: setup not complete, ignoring request")
        return
    Motors.roller.set_velocity(75, PERCENT)
    Motors.roller.spin(REVERSE)
    while Controllers.primary.buttonL1.pressing() or Controllers.secondary.buttonL1.pressing():
        wait(10)
    Motors.roller.stop()

    # Globals.ROLLER_ACTIVE = not Globals.ROLLER_ACTIVE
    # if Globals.ROLLER_ACTIVE:
    #     Motors.roller.set_velocity(75, PERCENT)
    #     Motors.roller.spin(REVERSE)
    # else:
    #     Motors.roller.stop()


# </editor-fold>


def start_loader() -> None:
    """
    A custom controller binding for starting the smart loader
    """
    if not Globals.SETUP_COMPLETE:
        bprint("[start_loader]: setup not complete, ignoring request")
        return
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
                if Globals.FLYWHEEL_AUTOSTART:
                    Globals.FLYWHEEL_START_TIME_MSEC = brain.timer.time(MSEC)
                    Globals.FLYWHEEL_STABLE = False
                    Globals.FLYWHEEL_ACTIVE = True
                    Motors.flywheel.set_velocity(Globals.FLYWHEEL_TARGET_POWER, PERCENT)
                    Motors.flywheel.spin(FORWARD)
                Motors.intake.set_velocity(100, PERCENT)
                Motors.intake.spin(FORWARD)
                while get_optical_color(Sensors.optical_disk) != Color.YELLOW and Globals.INTAKE_ACTIVE:
                    pass
                Motors.intake.stop()
                if Globals.INTAKE_ACTIVE:
                    Motors.intake.spin_for(REVERSE, 30, DEGREES)
                    Globals.DISK_READY = True
                    Globals.INTAKE_ACTIVE = False
            if Globals.INTAKE_ACTIVE and Globals.DISK_READY:
                Motors.intake.set_velocity(100, PERCENT)
                Motors.intake.spin_for(FORWARD, 360, DEGREES)
                Globals.INTAKE_ACTIVE = False
                Globals.DISK_READY = False
                if Globals.FLYWHEEL_AUTOSTART:  # Spin down the flywheel after a shot
                    Globals.FLYWHEEL_STABLE = False
                    Globals.FLYWHEEL_ACTIVE = False
                    Motors.flywheel.stop()


def reminder_handler() -> None:
    """
    Remind the user at certain time intervals by buzzing their controller
    """
    while not (competition.is_enabled() and competition.is_driver_control()) and Globals.SETUP_COMPLETE:
        wait(5)
    times_sorted = sorted(Globals.TIMERS)
    for reminder_time in times_sorted:
        while brain.timer.time(MSEC) - Globals.DRIVER_START_TIME_MSEC < reminder_time and competition.is_driver_control() and competition.is_enabled():
            pass
        if competition.is_driver_control() and competition.is_enabled():
            Controllers.secondary.rumble(Globals.TIMERS[reminder_time])
        else:
            break


def flywheel_spinup_haptics_handler():
    """
    Handle when to vibrate the controller because the flywheel is spun up
    """
    while not Globals.SETUP_COMPLETE:
        wait(100)  # It's okay to poll the settings thread at a lower rate for low priority tasks
    while True:
        while (not Globals.FLYWHEEL_ACTIVE) or Globals.FLYWHEEL_STABLE:
            wait(5)
        if Motors.flywheel.velocity(PERCENT) > Globals.FLYWHEEL_TARGET_POWER and brain.timer.time(MSEC) - Globals.FLYWHEEL_START_TIME_MSEC > Globals.FLYWHEEL_SPINUP_TIME_MSEC:
            Globals.FLYWHEEL_STABLE = True
            Controllers.primary.rumble("...")


def reset_loader() -> None:
    """
    Reset the state of the loader if it gets stuck
    """
    Globals.INTAKE_ACTIVE = False
    Globals.DISK_READY = False
    Globals.FLYWHEEL_ACTIVE = False
    Globals.PAUSE_LOADING_THREAD = False
    Motors.intake.stop()
    Motors.flywheel.stop()


def unload() -> None:
    """
    A custom controller binding for quickly reversing the roller and then reloading the current disk
    """
    if not Globals.SETUP_COMPLETE:
        bprint("[unload]: setup not complete, ignoring request")
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
        bprint("[unload_no_reload]: setup not complete, ignoring request")
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
    if not Globals.SETUP_COMPLETE:
        bprint("[fire_expansion]: setup not complete, ignoring request")
    Motors.expansion.set_max_torque(10, PERCENT)
    Motors.expansion.set_stopping(COAST)
    Motors.expansion.set_velocity(100, PERCENT)
    Motors.expansion.spin(REVERSE)
    wait(500)
    while Motors.expansion.velocity() > 10:
        wait(10)
    wait(400)
    Motors.expansion.stop()
    wait(1000)
    reset_expansion()


def reset_expansion() -> None:
    """
    Reset the expansion mechanism
    """
    Motors.expansion.set_max_torque(10, PERCENT)
    Motors.expansion.set_velocity(30, PERCENT)
    Motors.expansion.spin(FORWARD)
    Motors.expansion.set_stopping(COAST)  # Don't burn out the motor
    wait(100)
    while Motors.expansion.velocity() > 10:
        wait(10)
    Motors.expansion.spin_for(REVERSE, 10, DEGREES)
    Motors.expansion.set_stopping(HOLD)


# Primary controller bindings
Controllers.primary.buttonA.pressed(start_stop_flywheel)
Controllers.primary.buttonB.pressed(start_loader)
Controllers.primary.buttonX.pressed(unload)
Controllers.primary.buttonR2.pressed(unload_no_reload)
Controllers.primary.buttonL1.pressed(start_stop_roller)
Controllers.primary.buttonR1.pressed(reset_loader)
# Controllers.primary.buttonL2.pressed(fire_expansion)

# Secondary controller bindings
Controllers.secondary.buttonA.pressed(start_stop_flywheel)
Controllers.secondary.buttonB.pressed(start_loader)
Controllers.secondary.buttonX.pressed(unload)
Controllers.secondary.buttonR2.pressed(unload_no_reload)
Controllers.secondary.buttonL1.pressed(start_stop_roller)
Controllers.secondary.buttonR1.pressed(reset_loader)
Controllers.secondary.buttonY.pressed(reset_expansion)
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
    reminder_thread = Thread(reminder_handler)
    loading_thread = Thread(loading_handler)
    flywheel_haptics_thread = Thread(flywheel_spinup_haptics_handler)
    Globals.DRIVER_START_TIME_MSEC = brain.timer.time()
    while competition.is_driver_control() and competition.is_enabled():
        sleep(10)
    for thread in (driver_thread, reminder_thread, loading_thread, flywheel_haptics_thread):
        thread.stop()


# Register the competition functions
competition = Competition(driver_handler, autonomous_handler)
# </editor-fold>

if __name__ == "__main__":
    brain.screen.set_font(FontType.MONO12)
    if Controllers.primary.buttonX.pressing():  # If X is pressed on the primary controller then skip setup
        cclear()
        cprint("Skipped setup")
        Globals.STOPPING_MODE = COAST
        Globals.AUTONOMOUS_TASK = AutonomousTask.ROLLER_BOTH
        Globals.FLYWHEEL_AUTOSTART = True
    else:
        setup()
    # Apply the effect of seting Globals.STOPPING_MODE during setup
    Motors.allWheels.set_stopping(Globals.STOPPING_MODE)
    # Initialize a new smart drivetrain from our helper functions module (Not the vex one)
    drivetrain = BetterDrivetrain(inertial=Sensors.inertial, left_side=Motors.leftDrivetrain, right_side=Motors.rightDrivetrain, wheel_radius_mm=50, turn_aggression=0.25, correction_aggression=0.1, heading_offset_tolerance=1, motor_stall_speed=5, movement_slowdown_threshhold=500)
    cprint("Calibrating Gyro...")
    Sensors.inertial.calibrate()
    while Sensors.inertial.is_calibrating():
        pass
    cclear()
    Globals.SETUP_COMPLETE = True
    cprint("Setup complete")
    bprint("Setup complete")
    Controllers.primary.rumble(".")
    Controllers.secondary.rumble(".")
