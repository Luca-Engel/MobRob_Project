from tdmclient import ClientAsync, aw
from map.GridMap import GridMap
from thymio.MotionControl import rotation_nextpoint
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
        self.rotate_counter = 0
        self.circle_counter = 0
        self.speed_offset = 30

    
    def turn(self, dir):
        """
        Call this to turn left with 'l', right with 'r', or back with 'b'
        """
        if dir == 'r':
            motor_right_target = -100
            motor_left_target = 100
        elif dir == 'l':
            motor_right_target = 100
            motor_left_target = -100
        else:   #failsafe
            motor_right_target = -100
            motor_left_target = -100

        return motor_left_target, motor_right_target

    def update_prox(self, prox_horizontal):
        self.prox_horizontal = prox_horizontal
    def judge_severity(self):
        self.danger_state = DangerState.SAFE
        #Do no handle the back sensors, since Thymio does not reverse
        for i in range(5):
            if (self.prox_horizontal[i] > SensorThresh.STOP_THRESH.value):
                self.danger_dir[i] = DangerState.STOP
                self.danger_state = DangerState.STOP
            elif (self.prox_horizontal[i] > SensorThresh.WARN_THRESH.value):
                self.danger_dir[i] = DangerState.WARN
                if (self.danger_state.value < DangerState.WARN.value):
                    self.danger_state = DangerState.WARN
            else:
                self.danger_dir[i] = DangerState.SAFE
        return self.danger_state
    
    def first_rotation(self, direction, dir_changes, next_dir_change_idx):
        turn_dir = self.determine_turn_dir(direction, dir_changes, next_dir_change_idx)
        return self.turn(turn_dir)

    def determine_turn_dir(self, direction, dir_changes, next_dir_change_idx):
        previous_cell = np.array(dir_changes[next_dir_change_idx-1])
        current_cell = np.array(dir_changes[next_dir_change_idx])
        next_cell = np.array(dir_changes[next_dir_change_idx+1])

        current_vector = current_cell - previous_cell
        next_vector = next_cell - current_cell

        orientation_current_v = rotation_nextpoint(current_vector)
        orientation_next_v = rotation_nextpoint(next_vector)

        orientation_difference = (orientation_next_v - orientation_current_v)%360


        if orientation_difference < 180:
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
        self.rotate_counter = 0
        self.circle_counter = 0
    
    def run(self, direction, dir_changes, next_dir_change_idx, motor_speeds):
        if self.danger_state == DangerState.SAFE and self.state == LocalNavState.START:
            return motor_speeds
        
        #Danger
        if self.danger_state == DangerState.STOP and self.state == LocalNavState.START:
            self.state = LocalNavState.ROTATING #Start turning
            left_speed, right_speed = self.first_rotation(direction, dir_changes, next_dir_change_idx)
            return left_speed, right_speed
        
        if self.state == LocalNavState.ROTATING:
            if(self.danger_state == DangerState.SAFE):    #Done turning
                self.rotate_counter +=1
                if self.rotate_counter > 5:
                    self.state = LocalNavState.CIRCLING
                    return 0, 0
            return (self.first_rotation(direction, dir_changes, next_dir_change_idx))
        #Getting here means we are circling around the obstacle

        if self.state == LocalNavState.CIRCLING:
            self.circle_counter += 1
            if self.danger_state != DangerState.SAFE:
                motor_speeds = self.turn(self.turn_dir)
                self.circle_counter = 0     #Reset counter, we need to circle more
                return motor_speeds
            forward_speed = 100 - self.speed_offset
            if self.turn_dir == 'l':
                left_coeff = 1
                right_coeff = -0.8
            else:
                left_coeff = -0.8
                right_coeff = 1
            left_speed = forward_speed + left_coeff*self.speed_offset
            right_speed= forward_speed +right_coeff*self.speed_offset
            return left_speed, right_speed
        return motor_speeds

if __name__ == "__main__":
    local_nav = LocalNavigation()
    #Need to initialize map and Dijkstra nav,
    ClientAsync.run_async_program(local_nav.run)
