from tdmclient import ClientAsync
from thymio.MotionControl import rotation_nextpoint
import enum
import numpy as np

#Defines the threshold values for obstacle avoidance
class SensorThresh(enum.Enum):
    WARN_THRESH = 1000
    STOP_THRESH = 3000

#Lists danger states. Ascending order is increasing danger
class DangerState(enum.Enum):
    SAFE = 0
    WARN = 1
    STOP = 2


class LocalNavState(enum.Enum):
    START = 0
    ROTATING = 1
    CIRCLING = 2

#Variables for turning and circling state, determine the circling speed and radius
SPEED_OFFSET = 30
TURN_COEFF = 0.8
NORMAL_SPEED = 100
TURN_COUNTER_MAX = 5

class LocalNavigation:

    def __init__(self):
        self.prox_horizontal = [0, 0, 0, 0, 0, 0, 0]
        self.danger_state = DangerState.SAFE
        self.state = LocalNavState.START
        self.turn_dir = 0
        self.rotate_counter = 0
        self.circle_counter = 0
        #The circle_counter may be read outside the class, in order to decide on slack for circling

    
    def turn(self):
        """
        Reads a direction from the turn_dir field
        Call this to turn left with 'l', right with 'r'
        """
        if self.dir == 'r':
            motor_right_target = -NORMAL_SPEED
            motor_left_target = NORMAL_SPEED
        elif self.dir == 'l':
            motor_right_target = NORMAL_SPEED
            motor_left_target = -NORMAL_SPEED
        else:   #failsafe, do nothing
            motor_right_target = 0
            motor_left_target = 0

        return motor_left_target, motor_right_target

    def update_prox(self, prox_horizontal):
        """
        Updates the prox_horizontal variable with values read from the node (inaccessible here)
        """
        self.prox_horizontal = prox_horizontal


    def judge_severity(self):
        """
        Updates the danger_state of the LocalNavigation object and returns it
        """
        self.danger_state = DangerState.SAFE
        #Do no handle the back sensors, since Thymio does not reverse in Global Nav
        for i in range(5):
            if (self.prox_horizontal[i] > SensorThresh.STOP_THRESH.value):
                self.danger_state = DangerState.STOP
                return self.danger_state    #If any sensor is at STOP, we do not overwrite it
            elif (self.prox_horizontal[i] > SensorThresh.WARN_THRESH.value):
                self.danger_state = DangerState.WARN
        return self.danger_state
    
    def first_rotation(self, dir_changes, next_dir_change_idx):
        """
        Initiates a turn, returns the turn direction, to be called with motors()
        """
        self.determine_turn_dir(dir_changes, next_dir_change_idx)
        return self.turn()

    def determine_turn_dir(self, dir_changes, next_dir_change_idx):
        """
        Determines turn direction for circling an object, based on the Dijktra path.
        Since the path tends to stick to object, circling outside the next turn will
        avoid the Local obstacle without crossing a global obstacle.
        Updates turn_dir, to be called in turn()
        """
        previous_cell = np.array(dir_changes[next_dir_change_idx-1])
        current_cell = np.array(dir_changes[next_dir_change_idx])
        next_cell = np.array(dir_changes[next_dir_change_idx+1])

        current_vector = current_cell - previous_cell
        next_vector = next_cell - current_cell

        orientation_current_v = rotation_nextpoint(current_vector)
        orientation_next_v = rotation_nextpoint(next_vector)

        orientation_difference = (orientation_next_v - orientation_current_v)%360
        #We have now the turn angle of the next turn in the path, determine which way to turn to circle wide around it

        if orientation_difference < 180:
            self.turn_dir = 'r'
        else:
            self.turn_dir = 'l'

    def reset_state(self):
        """
        Reinitialises state and counters, to prepare for another obstacle
        """
        self.state = LocalNavState.START
        self.rotate_counter = 0
        self.circle_counter = 0
        self.turn_dir = 0
    
    def run(self, dir_changes, next_dir_change_idx, motor_speeds):
        """
        Handles obstacle avoidance with internal counters and state tracking.
        Expects to be called multiple times during obstacle avoidance.
        Must be reset once thymio is back on path with reset_state()
        Returns motor target speeds in a tuple
        """
        if self.danger_state == DangerState.SAFE and self.state == LocalNavState.START:
            return motor_speeds #Faulty call, return itself
        
        #Danger
        if self.danger_state == DangerState.STOP and self.state == LocalNavState.START:
            self.state = LocalNavState.ROTATING #Start turning
            left_speed, right_speed = self.first_rotation(dir_changes, next_dir_change_idx)
            return left_speed, right_speed 
        
        if self.state == LocalNavState.ROTATING:            #While rotating
            if(self.danger_state == DangerState.SAFE):    #If obstacle is cleared
                self.rotate_counter +=1                   #Keep turning to get some clearance
                if self.rotate_counter > TURN_COUNTER_MAX:                 #Done turning
                    self.state = LocalNavState.CIRCLING
                    return 0, 0
            
            #If not done turning, give the turn speeds
            return (self.first_rotation(dir_changes, next_dir_change_idx))
        
        #Getting here means we are circling around the obstacle

        if self.state == LocalNavState.CIRCLING:
            self.circle_counter += 1       #Counter for circling, gives some slack to the thymio to avoid staying in place
            
            if self.danger_state != DangerState.SAFE:   #Encounters another obstacle, or circle was too tight
                motor_speeds = self.turn()
                self.rotate_counter = 0  
                self.circle_counter = 0     #Reset counters, we need to start over
                return motor_speeds
            
            #We can circle safely
            forward_speed = NORMAL_SPEED - SPEED_OFFSET      #Fastest wheel turns at normal speed 
            if self.turn_dir == 'l':
                left_coeff = 1
                right_coeff = -TURN_COEFF
            else:
                left_coeff = -TURN_COEFF
                right_coeff = 1
            left_speed = forward_speed + left_coeff*SPEED_OFFSET
            right_speed= forward_speed +right_coeff*SPEED_OFFSET
            return left_speed, right_speed
        return motor_speeds

if __name__ == "__main__":
    #Here for testing purposes
    local_nav = LocalNavigation()
    #Need to initialize map and Dijkstra nav,
    ClientAsync.run_async_program(local_nav.run)
