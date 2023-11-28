from tdmclient import ClientAsync
from map.GridMap import GridMap
import enum
import numpy as np
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


Wl = [40, 20, -20, -20, -40]
Wr = [-40, -20, -20, 20, 40]

class LocalNavigation:

    def __init__(self):
        # Add the map, the thymio position and direction
        # self.target_dir = 0
        self.prox_horizontal = [0, 0, 0, 0, 0, 0, 0]
        self.danger_state = DangerState.SAFE
        self.prev_danger_state = DangerState.SAFE
        self.danger_dir = [DangerState.SAFE]*7
        self.target_dir = 0 #should express angle of movement, between -180 and 179, positive to the left

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
        danger_state = DangerState.SAFE

        print("Value", prox_horizontal[1])
        for i in range(5):
            if (prox_horizontal[i] > SensorThresh.STOP_THRESH):
                self.danger_dir[i] = DangerState.STOP
                danger_state = DangerState.STOP
            elif (prox_horizontal[i] > SensorThresh.WARN_THRESH):
                self.danger_dir[i] = DangerState.WARN
                if (danger_state < DangerState.WARN):
                    danger_state = DangerState.WARN
            else:
                self.danger_dir[i] = DangerState.SAFE
        return danger_state

    def danger_nav(self):
        # Simple cases, one side is totally safe, the other is not
        if (self.danger_dir[SensorPos.LEFT_SENSOR] == DangerState.SAFE):

            left_speed, right_speed = self.turn('l')

        elif (self.danger_dir[SensorPos.RIGHT_SENSOR] == DangerState.SAFE):
            left_speed, right_speed = self.turn('r')

        # Consider target direction, check if it safe to go there
        # If not, enter backwards mode
        if (self.target_dir < 0):
            if (self.danger_dir[SensorPos.RIGHT_SENSOR] < DangerState.STOP):
                left_speed, right_speed = self.turn('r')
            else:
                left_speed, right_speed = self.turn('b')

        else:
            if (self.danger_dir[SensorPos.LEFT_SENSOR] < DangerState.STOP):
                left_speed, right_speed = self.turn('l')

            else:
                left_speed, right_speed = self.turn('b')

        # Just here for sanity
        left_speed, right_speed = self.turn('b')
        return left_speed, right_speed

    def potential_field(self, prox_horizontal, motor_left_target, motor_right_target):
        # This isn't helping, we need to convert this into a "slightly steer away" if possible
        # Otherwise, just throw this away, it's not vital, and lower STOP threshold slightly
        x = prox_horizontal

        y1 = 0
        y2 = 0
        for i in range(2):
            y1 += Wl[i] * x[i] / 200        #contribute to left motor the left sensors
            y2 += Wr[i+3] * x[i+3] / 200    #contribute to right motor the right sensors
        motor_left_target = motor_left_target / 2 + y1 - y2
        motor_right_target = motor_right_target / 2 + y2 - y1

        return motor_left_target, motor_right_target

    def motors(self, speed_left, speed_right):
        return {
            "motor.left.target": [speed_left], "motor.right.target": [speed_right],
        }

    async def run(self):
        # Connecting the thymio
        Client = ClientAsync()
        node = await Client.wait_for_node()
        await node.lock()

        while True:
            await node.wait_for_variables()
            self.prox_horizontal = node["prox.horizontal"]
            #left_speed, right_speed = get_current_motors, we need it
            self.prev_danger_state = self.danger_state
            self.danger_state = self.judge_severity(self.prox_horizontal)
            if(self.danger_state != DangerState.SAFE and self.prev_danger_state == DangerState.SAFE):
                #memorize current goal, set a flag blocking Global Nav
                print("thymio enters unsafe waters")
            if (self.danger_state == DangerState.STOP):  # Override all other nav, thymio needs to turn to safety
                
                left_speed, right_speed = self.danger_nav()
                print("danger")

            elif (self.danger_state == DangerState.WARN):  # Add potential field vector to whatever nav we used before
                left_speed, right_speed = self.potential_field(self.prox_horizontal, left_speed, right_speed)
                print("warn")

            elif (self.danger_state == DangerState.SAFE and self.prev_danger_state != DangerState.SAFE):
                # Check Thymio position, take the path array, compute closest unvisited goal, and mark the skipped ones as visited.
                # call reorientation routine
                # clear the flag blocking Global Nav
                print("back in the game")
            print("Speed", left_speed, right_speed)
            await node.set_variables(self.motors(int(left_speed), int(right_speed)))

            await Client.sleep(0.1)



if __name__ == "__main__":
    local_nav = LocalNavigation()
    #Need to initialize map and Dijkstra nav,
    ClientAsync.run_async_program(local_nav.run)
