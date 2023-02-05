"""
Helper functions for our program
"""


def cubic_normalize(value: float, linearity: float) -> float:
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


def controller_input_to_motor_power(controller_l: tuple, controller_r: tuple, linearity: float) -> tuple:
    """
    Convert raw input from a controller into left and right motor output values
    :param linearity: How linear the cubic_normalize function should convert the powers
    :type linearity: float
    :param controller_l: The left stick's (X, Y) pair
    :type controller_l: tuple[float, float]
    :param controller_r: The left stick's (X, Y) pair
    :type controller_r: tuple[float, float]
    :rtype: tuple[float, float]
    :returns: A tuple of (left_motor_power, right_motor_power)
    """
    controller_l_x, controller_l_y = controller_l
    controller_r_x, controller_r_y = controller_r
    controller_l_y = cubic_normalize(controller_l_y() * 0.01, linearity)
    controller_r_y = cubic_normalize(controller_r_y() * 0.01, linearity)
    left_speed, right_speed = controller_l_y, controller_r_y
    return left_speed * -100, right_speed * -100


# noinspection PyTypeChecker
# ^ Prevents linter from getting mad about reporting an int but returning an Optional[int] with no "typing" library on the vex brain's python version
def get_optical_color(optical_sensor: Optical) -> int:
    """
    Get the color currently detected by the passed optical sensor
    :param optical_sensor: The sensor to read from
    :type optical_sensor: Optical
    :rtype: int
    :returns: Color.RED, Color.BLUE, or None
    """
    optical_hue = optical_sensor.hue()
    if optical_hue < 30 or optical_hue > 340:
        return Color.RED
    elif 215 < optical_hue < 255:
        return Color.BLUE
    elif 28 < optical_hue < 50:
        return Color.YELLOW
    return


class BetterDrivetrain:
    """
    A better drivetrain than the default Vex one
    """

    def __init__(self, inertial: Inertial, left_side: MotorGroup, right_side: MotorGroup,
                 heading_offset_tolerance: float, turn_aggression: float, correction_aggression: float,
                 wheel_radius_mm: float) -> None:
        """
        Initialize a new drivetrain with the specified properties
        :param inertial: The inertial sensor to use for the drivetrain
        :param left_side: The motor/motor group corresponding to the left side of the robot
        :param right_side: The motor/motor group corresponding to the right side of the robot
        :param heading_offset_tolerance: The delta heading that is acceptable or close enough
        :param turn_aggression: How aggressive to be while turning
        :param correction_aggression: How aggressive to be while moving
        :param wheel_radius_mm: The radis of the wheels
        """
        self.inertial = inertial
        self.left_side = left_side
        self.drivetrain_motors = MotorGroup(left_side, right_side)
        self.right_side = right_side
        self.heading_offset_tolerance = heading_offset_tolerance
        self.turn_aggression = turn_aggression
        self.correction_aggression = correction_aggression
        self.wheel_radius_mm = wheel_radius_mm
        self.wheel_circumference_mm = 2 * wheel_radius_mm * 3.141592
        self.current_heading = 0

    def turn_to_heading(self, heading: float) -> None:
        """
        Turn to an absolute heading using the inertial sensor
        :param heading: The absolute heading to turn to
        """
        heading %= 360
        current_heading = self.inertial.heading(DEGREES) % 360
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
        self.drivetrain_motors.set_velocity(0, PERCENT)
        self.drivetrain_motors.spin(FORWARD)
        while abs(delta_heading) > heading_offset_tolerance:
            if left_turn_difference < right_turn_difference:
                delta_heading = left_turn_difference
                self.left_side.set_velocity(delta_heading * turn_aggression + 5, PERCENT)
                self.right_side.set_velocity((delta_heading * turn_aggression + 5) * -1, PERCENT)
            else:
                delta_heading = right_turn_difference
                self.left_side.set_velocity((delta_heading * turn_aggression + 5) * -1, PERCENT)
                self.right_side.set_velocity(delta_heading * turn_aggression + 5, PERCENT)
            current_heading = self.inertial.heading(DEGREES) % 360
            left_turn_difference = (current_heading - heading)
            right_turn_difference = (heading - current_heading)
            if left_turn_difference < 0:
                left_turn_difference = left_turn_difference + 360
            if right_turn_difference < 0:
                right_turn_difference = right_turn_difference + 360
        self.drivetrain_motors.stop()
        self.current_heading = heading

    def move_towards_heading(self, heading: float, speed: float, distance_mm: float) -> None:
        """
        Move towards a heading using dynamic course correction
        :param heading: The absolute heading to move towards
        :param speed:  The base speed to move at
        :param distance_mm: The distance to move before stopping the movement
        """
        heading %= 360
        initial_distance_traveled = (((
                                              self.left_side.position(DEGREES) + self.right_side.position(DEGREES)) / 2) / 360) * self.wheel_circumference_mm
        distance_traveled = abs((((
                                          self.left_side.position(DEGREES) + self.right_side.position(DEGREES)) / 2) / 360) * self.wheel_circumference_mm - initial_distance_traveled)
        self.drivetrain_motors.set_velocity(0, PERCENT)
        self.drivetrain_motors.spin(FORWARD)
        while distance_traveled < distance_mm:
            distance_traveled = abs((((
                                              self.left_side.position(DEGREES) + self.right_side.position(DEGREES)) / 2) / 360) * self.wheel_circumference_mm - initial_distance_traveled)
            current_heading = self.inertial.heading(DEGREES) % 360
            left_turn_difference = (current_heading - heading)
            right_turn_difference = (heading - current_heading)
            if left_turn_difference < 0:
                left_turn_difference += 360
            if right_turn_difference < 0:
                right_turn_difference += 360
            if left_turn_difference < right_turn_difference:  # Turn towards the most efficient direction
                delta_heading = left_turn_difference
                self.left_side.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
                self.right_side.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
            else:
                delta_heading = right_turn_difference
                self.left_side.set_velocity((delta_heading * turn_aggression - speed) * -1, PERCENT)
                self.right_side.set_velocity(delta_heading * turn_aggression + speed, PERCENT)
        self.drivetrain_motors.stop()
        self.current_heading = heading

    def move_with_controller(self) -> None:
        """
        Move using the controller input
        """
        left_speed, right_speed = controller_input_to_motor_power(
            (self.controller.axis4.position, self.controller.axis3.position),
            (self.controller.axis1.position, self.controller.axis2.position),
            linearity=Globals.SPEED_CURVE_LINEARITY)
        self.left_side.set_velocity((left_speed + left_offset), PERCENT)
        self.right_side.set_velocity((right_speed + right_offset), PERCENT)


class CustomPID:
    """
    Wrap a motor definition in this class to use a custom PID to control its movements ie: my_motor = CustomPID(Motor(...), kp, kd, t)
    Waring, this class disables all motor functionality except the following functions:[set_velocity, set_stopping, stop, spin, velocity]
    :param motor_object: The motor to apply the PID to
    :param kp: Kp value for the PID: How quickly to modify the speed if it has not yet reached the desired speed
    :param kd: Kd value for the PID: Reduces the speed of response and limit overshoot
    :param t: Time between PID updates
    """

    def __init__(self, motor_object, kp: float = 0.4, kd: float = 0.05, t: float = 0.01) -> None:
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

    def PID_loop(self) -> None:
        """
        Used to run in a thread using "Thread(<motor to run PID on>.PID_loop)"; it updates the values the PID uses and
        handles applying new normalized speed values to the motor
        :return: None
        """
        while True:
            self.PID_update()
            wait(self.t, SECONDS)

    def set_velocity(self, velocity, unit) -> None:
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

    def set_stopping(self, **kwargs) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.set_stopping(kwargs)

    def stop(self) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.stop()

    def spin(self, direction) -> None:
        """
        Passthrough method to the "Motor" class
        """
        self.motor_object.spin(direction)

    def velocity(self, *args) -> float:
        """
        Passthrough method to the "Motor" class
        """
        return self.motor_object.velocity(*args)


class ToggleMotor:
    """
    Wrap a motor in this class to allow switching it between several speeds
    """

    def __init__(self, motor, speeds, event):
        self.motor_object = motor
        self.states = list(speeds)
        self.current_state = 0
        self.button = event

    def next_state(self):
        """
        Move the motor to the next state in the list
        """
        self.current_state += 1
        if self.current_state >= len(self.states):
            self.current_state = 0
        self.motor_object.set_velocity(self.states[self.current_state])

    def setup_handler(self):
        """
        Bind the passed button to our "next_state" callback
        """
        self.button(self.next_state)
