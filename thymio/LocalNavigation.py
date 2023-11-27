from tdmclient import ClientAsync
import cv2

target_dir = 100  # positive to the right
# Every time the prox event is generated, the robot go back accordingly
# of what is sensed on the middle front proximity sensor
LEFT_SENSOR = 0
LEFT_CENTER_SENSOR = 1
FRONT_SENSOR = 2
RIGHT_CENTER_SENSOR = 3
RIGHT_SENSOR = 4
prox_horizontal = [0, 0, 0, 0, 0, 0, 0]

# unused, still here
LEFT_BACK = 5
RIGHT_BACK = 6

WARN_THRESH = 1000
STOP_THRESH = 3000

SAFE = 0
WARN = 1
STOP = 2

danger_state = 0
danger_dir = [0, 0, 0, 0, 0]

Wl = [40, 20, -20, -20, -40]
Wr = [-40, -20, -20, 20, 40]

l = 1
r = 2
b = 3

toggle0 = 0
toggle1 = 0

motor_left_target = 100
motor_right_target = 100

class LocalNavigation:

    def __init__(self):
        self.target_dir = 100
        self.prox_horizontal = [0, 0, 0, 0, 0, 0, 0]
        self.danger_state = 0
        self.danger_dir = [0, 0, 0, 0, 0]
        self.motor_left_target = 100
        self.motor_right_target = 100

    def turn(self, dir):
        if dir == r:
            motor_right_target = -80
            motor_left_target = 80
        elif dir == l:
            motor_right_target = 80
            motor_left_target = -80
        else:
            motor_right_target = -150
            motor_left_target = -150

        return motor_left_target, motor_right_target

    def judge_severity(self, prox_horizontal):
        danger_state = SAFE

        print("Value", prox_horizontal[1])
        for i in range(5):
            if (prox_horizontal[i] > STOP_THRESH):
                danger_dir[i] = STOP
                danger_state = STOP
            elif (prox_horizontal[i] > WARN_THRESH):
                danger_dir[i] = WARN
                if (danger_state < WARN):
                    danger_state = WARN
            else:
                danger_dir[i] = SAFE
        return danger_state

    def danger_nav(self):
        # Simple cases, one side is totally safe, the other is not
        if (danger_dir[LEFT_SENSOR] == SAFE):

            left_speed, right_speed = self.turn(l)

        elif (danger_dir[RIGHT_SENSOR] == SAFE):
            left_speed, right_speed = self.turn(r)

        # Consider target direction, check if it safe to go there
        # If not, enter backwards mode
        if (target_dir > 0):
            if (danger_dir[RIGHT_SENSOR] < STOP):
                left_speed, right_speed = self.turn(r)
            else:
                left_speed, right_speed = self.turn(b)

        else:
            if (danger_dir[LEFT_SENSOR] < STOP):
                left_speed, right_speed = self.turn(l)

            else:
                left_speed, right_speed = self.turn(b)

        # Just here for sanity
        left_speed, right_speed = self.turn(b)
        return left_speed, right_speed

    def potential_field(self, prox_horizontal, motor_left_target, motor_right_target):
        x = prox_horizontal

        y1 = 0
        y2 = 0
        for i in range(5):
            y1 += Wl[i] * x[i] // 200
            y2 += Wr[i] * x[i] // 200
        motor_left_target = motor_left_target // 2 + y1
        motor_right_target = motor_right_target // 2 + y2

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
            prox_horizontal = node["prox.horizontal"]

            danger_state = self.judge_severity(prox_horizontal)

            if (danger_state == STOP):  # Override all other nav, thymio needs to turn to safety

                left_speed, right_speed = self.danger_nav()
                print("danger")

            elif (danger_state == WARN):  # Add potential field vector to whatever nav we used before

                left_speed, right_speed = self.potential_field(prox_horizontal, 100, 100)
                print("potential")

            else:  # Safe nav, no control needed

                left_speed = 100
                right_speed = 100
                print("safe")

            print("Speed", left_speed, right_speed)
            await node.set_variables(self.motors(int(left_speed), int(right_speed)))

            await Client.sleep(0.1)



if __name__ == "__main__":
    local_nav = LocalNavigation()
    ClientAsync.run_async_program(local_nav.run)
