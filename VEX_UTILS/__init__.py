"""
A module with all the helper functions required by our program
"""


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
