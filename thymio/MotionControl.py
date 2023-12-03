import math

from tdmclient import ClientAsync, aw


# Create a class for every instance of the motors
class Motion:
    def __init__(self, node):

        self.node = node

        self.changing_pose = False
        self.nextpoint_achieved = False

        self.Kp = 3
        self.Ki = 0.2

        self.distance = 0
        self.angle = 0
        self.normal_speed = 100

        self.threshold_angle = 2
        self.desired_angle = 0
        # Need to calibrate this for the lecture of the angle
        self.calibration = 5

        self.sum_error = 0
        self.max_sum_error = 30
        self.error = 0
        self.change_idx = -1

        self._total_actual_angle = None
        self._last_actual_angle = None
        self._total_wanted_angle = None
        self._last_wanted_angle = None

    def motors(self, speed_left, speed_right):

        return {
            "motor.left.target": [int(speed_left)], "motor.right.target": [int(speed_right)],
        }

    def move(self, left_speed, right_speed):
        aw(self.node.send_set_variables(self.motors(int(left_speed), int(right_speed))))

    def pi_regulation(self, actual_angle, wanted_angle, position, change_idx):
        self.desired_angle = wanted_angle

        # Position at 0 means that the robot achieved the next point and need to stop moving
        if position == 0:
            left_speed = 0
            right_speed = 0
            return left_speed, right_speed
            # position = 2

        position = 1


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
            position = 2
        elif change_idx != self.change_idx:
            self.change_idx = change_idx
        elif abs(self._total_wanted_angle - self._total_wanted_angle) > 30:
            self.change_idx = -1
            position = 2

        # Position at 1 means that the robot is achieving the next point
        if position == 1:
            left_speed = 0
            right_speed = 0
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

                if self.sum_error < self.max_sum_error:
                    left_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.sum_error)
                    right_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.sum_error)

                else:
                    left_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.max_sum_error)
                    right_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.max_sum_error)

            elif self._total_actual_angle > (self._total_wanted_angle + self.threshold_angle):
                self.error = abs(self._total_actual_angle - self._total_wanted_angle)
                self.sum_error += self.error

                if self.sum_error < self.max_sum_error:
                    left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.sum_error)
                    right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.sum_error)

                else:
                    left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.max_sum_error)
                    right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.max_sum_error)

            return left_speed, right_speed
        # Position at 2 means that the robot need to rotate to face the next point
        elif position == 2:

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
                position = 0
                return 0, 0


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



async def motion_control_test():
    # Connecting the thymio
    Client = ClientAsync()
    node = await Client.wait_for_node()
    await node.lock()
    MotorControl = Motion(node)

    MotorControl.move(0, 0)


if __name__ == "__main__":
    ClientAsync.run_async_program(motion_control_test)
