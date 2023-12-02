from tdmclient import ClientAsync, aw
from map.GridMap import GridMap
import enum
import numpy as np
import math
import cv2

# Every time the prox event is generated, the robot go back accordingly
# of what is sensed on the middle front proximity sensor

class SensorPos(enum.Enum):
    LEFT_SENSOR = 0
    LEFT_CENTER_SENSOR = 1
    FRONT_SENSOR = 2
    RIGHT_CENTER_SENSOR = 3
    RIGHT_SENSOR = 4
    LEFT_BACK = 5
    RIGHT_BACK = 6

    def __str__(self):
        return self.value

class SensorThresh(enum.Enum):
    WARN_THRESH = 1000
    STOP_THRESH = 3000

    def __str__(self):
        return self.value

class DangerState(enum.Enum):
    SAFE = 0
    WARN = 1
    STOP = 2

class LocalNavState(enum.Enum):
    START = 0
    ROTATING = 1
    CIRCLING = 2


class LocalNavigation:

    def __init__(self):
        self.prox_horizontal = [0, 0, 0, 0, 0, 0, 0]
        self.danger_state = DangerState.SAFE
        self.danger_dir = [DangerState.SAFE]*7
        self.state = LocalNavState.START
        self.turn_dir = 0

    
    def turn(self, dir):
        """
        Call this to turn left with 'l', right with 'r', or back with 'b'
        """
        if dir == 'r':
            motor_right_target = -80
            motor_left_target = 80
        elif dir == 'l':
            motor_right_target = 80
            motor_left_target = -80
        else:
            motor_right_target = -150
            motor_left_target = -150

        return motor_left_target, motor_right_target

    
    def judge_severity(self, prox_horizontal):
        self.danger_state = DangerState.SAFE
        print("Value", prox_horizontal[1])
        #Do no handle the back sensors, since Thymio does not reverse
        for i in range(5):
            if (prox_horizontal[i] > SensorThresh.STOP_THRESH):
                self.danger_dir[i] = DangerState.STOP
                self.danger_state = DangerState.STOP
            elif (prox_horizontal[i] > SensorThresh.WARN_THRESH):
                self.danger_dir[i] = DangerState.WARN
                if (self.danger_state < DangerState.WARN):
                    self.danger_state = DangerState.WARN
            else:
                self.danger_dir[i] = DangerState.SAFE

    
    def first_rotation(self, direction, dir_changes, next_dir_change_idx):
        turn_dir = self.determine_turn_dir(direction, dir_changes, next_dir_change_idx)
        return self.turn(turn_dir)

    def determine_turn_dir(self, direction, dir_changes, next_dir_change_idx):
        previous_cell = dir_changes[next_dir_change_idx-1]
        current_cell = dir_changes[next_dir_change_idx]
        turn_abs_vector = np.subtract(previous_cell, current_cell)
        turn_vector = np.subtract(direction, turn_abs_vector)
        turn_degrees = math.atan2(turn_vector[0], turn_vector[1]) / math.pi * 180
        turn_degrees = turn_degrees%360
        if turn_degrees > 180:
            self.turn_dir = 'r'
            return 'r'
        else:
            self.turn_dir = 'l'
            return 'l'
        

    def motors(self, speed_left, speed_right):
        return {
            "motor.left.target": [speed_left], "motor.right.target": [speed_right],
        }

    def reset_state(self):
        self.state = LocalNavState.START
    
    async def run(self, direction, dir_changes, next_dir_change_idx, node):
        self.prox_horizontal = node["prox.horizontal"]
        if self.danger_state == DangerState.SAFE and self.state == LocalNavState.START:
            return        #failsafe
        
        #Danger
        if self.danger_state != DangerState.SAFE and self.state == LocalNavState.START:
            self.state = LocalNavState.ROTATING #Start turning
        
        if self.state == LocalNavState.ROTATING:
            left_speed, right_speed = self.first_rotation(direction, dir_changes, next_dir_change_idx)
            aw(node.set_variables(self.motors(int(left_speed), int(right_speed))))
            self.judge_severity(node["prox_horizontal"])
            if(self.danger_state == LocalNavState.SAFE):    #Done turning
                self.state = LocalNavState.CIRCLING
            return 
        #Getting here means we are circling around the obstacle
        forward_speed = 200
        speed_offset  = 100

        if self.state == LocalNavState.CIRCLING:
            if self.danger_state != DangerState.SAFE:
                speed_offset -=10
            if self.turn_dir == 'l':
                left_coeff = -1
                right_coeff = 1
            else:
                left_coeff = 1
                right_coeff = -1
            left_speed = forward_speed + left_coeff*speed_offset
            right_speed= forward_speed +right_coeff*speed_offset
            aw(node.set_variables(self.motors(int(left_speed), int(right_speed))))


if __name__ == "__main__":
    local_nav = LocalNavigation()
    #Need to initialize map and Dijkstra nav,
    ClientAsync.run_async_program(local_nav.run)
