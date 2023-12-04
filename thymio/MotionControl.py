import math

from tdmclient import ClientAsync, aw

# Constant in motion control
STOP_MOVING = 0
STOP = 0
MOVE = 1
ROTATE = 2

# Create a class for every instance of the motors
class Motion:
    def __init__(self, node):

        self.node = node

        self.changing_pose      = False
        self.nextpoint_achieved = False
        
        self.distance     = 0
        self.angle        = 0
        self.normal_speed = 100
        
        self.threshold_angle = 2 # Avoiding oscillation on the desired angle
        self.desired_angle   = 0
        # Need to calibrate this for the lecture of the angle
        self.calibration     = 5

        # Regulator parameter
        self.Kp            = 3         # Finding the parameters of our Pi controller by tuning them during test
        self.Ki            = 0.2
        self.sum_error     = 0
        self.max_sum_error = 30
        self.error         = 0
        self.change_idx    = -1
        
        # Parameters of the Thymio angle and the desired one
        self._total_actual_angle = None
        self._last_actual_angle = None
        self._total_wanted_angle = None
        self._last_wanted_angle = None

    def motors(self, speed_left, speed_right):
        """""
        Send the information to provide to the Thymio for changing the speed of the wheels
        Return : "motor.left.target": [int(speed_left)], "motor.right.target": [int(speed_right)]
    """
        return {
            "motor.left.target": [int(speed_left)], "motor.right.target": [int(speed_right)],
        }

    def move(self, left_speed, right_speed):
    """""
        Send the new velocity of the motors to the Thymio
    """
        aw(self.node.send_set_variables(self.motors(int(left_speed), int(right_speed))))

    def pi_regulation(self, actual_angle, wanted_angle, movement, change_idx):
    """""
        The control of the robot is done in this function. We have 3 states that the Thymio can be
        1) STOP: the Thymio has reached a point so we stop for a instance the motors
        2) MOVE: the Thymio is moving to the next point with a PI controller that adjust the velocity
                 of both wheels to adjust the trajectory of the robot
        3) ROTATE: Before moving to the next point, the Thymio rotate on itself to face the next point
                   it needs to go
        Return : left_speed, right_speed
    """
        self.desired_angle = wanted_angle

        # Movement at Stop means that the robot has achieved the next point and stop moving
        if movement == STOP:
            left_speed = STOP_MOVING
            right_speed = STOP_MOVING
            return left_speed, right_speed

        # Initialise the movement of the Thymio
        movement = MOVE

        # total angle stuff ---------
        if self._total_actual_angle is None:
            self._total_actual_angle = actual_angle
            self._last_actual_angle = actual_angle

            self._total_wanted_angle = wanted_angle
            self._last_wanted_angle = wanted_angle

        else:
            if abs(actual_angle - self._last_actual_angle) > 180:
                if actual_angle > self._last_actual_angle:
                    actual_angle -= 360
                else:
                    actual_angle += 360

            if abs(wanted_angle - self._last_wanted_angle) > 180:
                if wanted_angle > self._last_wanted_angle:
                    wanted_angle -= 360
                else:
                    wanted_angle += 360

            difference_actual_angle = actual_angle - self._last_actual_angle
            difference_wanted_angle = wanted_angle - self._last_wanted_angle

            self._total_actual_angle += difference_actual_angle
            self._total_wanted_angle += difference_wanted_angle

            while abs(self._total_actual_angle - self._total_wanted_angle) > 360:
                if self._total_actual_angle > self._total_wanted_angle:
                    self._total_actual_angle -= 360
                else:
                    self._total_actual_angle += 360

            self._last_actual_angle = actual_angle
            self._last_wanted_angle = wanted_angle

        if (change_idx != self.change_idx and abs(self._total_actual_angle - self._total_wanted_angle) > 10):  # < 6 degrees
            movement = ROTATE
        elif change_idx != self.change_idx:
            self.change_idx = change_idx
        elif abs(self._total_wanted_angle - self._total_wanted_angle) > 30:
            self.change_idx = -1
            movement = ROTATE

        # movement at Move means that the robot is achieving the next point
        if movement == MOVE:
            left_speed = STOP_MOVING
            right_speed = STOP_MOVING

            # Control if the Thymio is inside the threshold cone of acceptance. If it's not the case
            # the pi controller is active to correct the trajectory
            if ((self._total_wanted_angle - self.threshold_angle) <= self._total_actual_angle
                    and self._total_actual_angle <= (self._total_wanted_angle + self.threshold_angle)):
                left_speed = self.normal_speed
                right_speed = self.normal_speed

                self.error = 0
                self.sum_error = 0
            
            elif self._total_actual_angle < (self._total_wanted_angle - self.threshold_angle):
                self.error = abs(self._total_actual_angle - self._total_wanted_angle)
                if self.error > 330:
                    self.error = 360 - self.error

                self.sum_error += self.error

                # To avoid that the Thymio start to drift with high speed in a wheel, we implemented an anti wind-up
                # to protect from the integrator of our controller
                if self.sum_error < self.max_sum_error:
                    left_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.sum_error)
                    right_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.sum_error)

                else:
                    left_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.max_sum_error)
                    right_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.max_sum_error)

            elif self._total_actual_angle > (self._total_wanted_angle + self.threshold_angle):
                self.error = abs(self._total_actual_angle - self._total_wanted_angle)
                self.sum_error += self.error

                # To avoid that the Thymio start to drift with high speed in a wheel, we implemented an anti wind-up
                # to protect from the integrator of our controller
                if self.sum_error < self.max_sum_error:
                    left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.sum_error)
                    right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.sum_error)

                else:
                    left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.max_sum_error)
                    right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.max_sum_error)

            return left_speed, right_speed
            
        # Movement at Rotate means that the robot need to rotate to face the next point
        elif Movement == ROTATE:

            if (self._total_wanted_angle - self._total_actual_angle) >= 0:
                if self._total_wanted_angle - self._total_actual_angle >= 180:
                    left_speed = self.normal_speed
                    right_speed = -self.normal_speed
                else:
                    left_speed = -self.normal_speed
                    right_speed = self.normal_speed
            elif (self._total_wanted_angle - self._total_actual_angle) < 0:
                if (self._total_actual_angle - self._total_wanted_angle) > 180:
                    left_speed = -self.normal_speed
                    right_speed = self.normal_speed
                else:
                    left_speed = self.normal_speed
                    right_speed = -self.normal_speed
            return left_speed, right_speed

            if self.error >= 0:
                left_speed = self.normal_speed
                right_speed = -self.normal_speed
                return left_speed, right_speed

            if self.error < 0:
                left_speed = -self.normal_speed
                right_speed = self.normal_speed
                return left_speed, right_speed

            else:
                movement = STOP
                return STOP_MOVING, STOP_MOVING


def distance_nextpoint(actual_pos, next_pos):
    """""
        Calculate the distance to the next point in
        the global navigation
    """
    return math.sqrt(math.pow((actual_pos[0] - next_pos[0]), 2) + math.pow(actual_pos[1] - next_pos[1], 2))


def rotation_nextpoint(direction):
    """""
        Rotation of the Thymio before moving to
        the next point
        Return the value in degree
    """
    return math.atan2(direction[0], direction[1]) / math.pi * 180


# Function to test the motors of the Thymio without the rest of the code
async def motion_control_test():
    # Connecting the thymio
    Client = ClientAsync()
    node = await Client.wait_for_node()
    await node.lock()
    MotorControl = Motion(node)

    MotorControl.move(0, 0)


if __name__ == "__main__":
    ClientAsync.run_async_program(motion_control_test)
