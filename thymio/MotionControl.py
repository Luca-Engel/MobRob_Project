import time
import numpy as np
from tdmclient import ClientAsync
import math


# Create a class for every instance of the motors
class Motion:
    def __init__(self, node):

        self.node = node

        self.changing_pose = False
        self.nextpoint_achieved = False

        self.Kp = 1
        self.Ki = 0.1

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

    def motors(self, speed_left, speed_right):

        return {
            "motor.left.target": [speed_left], "motor.right.target": [speed_right],
        }

    def move(self, left_speed, right_speed):

       self.node.send_set_variables(self.motors(left_speed, right_speed))

    def pi_regulation(self, actual_angle, position):

        # Position at 0 means that the robot achieved the next point and need to stop moving
        if position == 0:
            self.move(left_speed = 0, right_speed = 0)
            position = 2

        # Position at 1 means that the robot is achieving the next point
        elif position == 1:

            if actual_angle <= abs(self.desired_angle + self.threshold_angle):
                left_speed = self.normal_speed
                right_speed = self.normal_speed

                self.error = 0
                self.sum_error = 0

            elif actual_angle > (self.desired_angle + self.threshold_angle):
                self.error = abs(actual_angle - self.desired_angle)
                self.sum_error += self.error

            elif actual_angle < (self.desired_angle - self.threshold_angle):
                self.error = abs(actual_angle - self.desired_angle)
                self.sum_error += self.error

            if self.sum_error < self.max_sum_error:
                left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.sum_error)
                right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.sum_error)
            else:
                left_speed = self.normal_speed + (self.Kp * self.error + self.Ki * self.max_sum_error)
                right_speed = self.normal_speed - (self.Kp * self.error + self.Ki * self.max_sum_error)

            self.move(left_speed, right_speed)
        # Position at 2 means that the robot need to rotate to face the next point
        elif position == 2:
            self.error = abs(actual_angle - self.desired_angle)

            if self.error < self.desired_angle - self.threshold_angle:
                self.move(self.normal_speed, -self.normal_speed)
            if self.error > self.desired_angle + self.threshold_angle:
                self.move(-self.normal_speed, self.normal_speed)
            else:
                position = 0


def distance_nextpoint(actual_pos, next_pos):
    """""
        Calculate the distance to the next point in
        the global navigation
    """
    return math.sqrt(math.pow((actual_pos[0]-next_pos[0]),2) + math.pow(actual_pos[1]-next_pos[1],2))


def rotation_nextpoint(actual_pos, next_pos):
    """""
        Rotation of the Thymio before moving to
        the next point
        Return the value in degree
    """
    return math.atan2(actual_pos[0]-next_pos[0], actual_pos[1]-next_pos[1])/math.pi*180


async def main():
    # Connecting the thymio
    Client = ClientAsync()
    node = await Client.wait_for_node()
    await node.lock()
    MotorControl = Motion(node)

    MotorControl.move(0, 0)


if __name__ == "__main__":
    ClientAsync.run_async_program(main)
