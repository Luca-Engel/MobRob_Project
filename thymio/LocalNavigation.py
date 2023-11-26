%%run_python

timer_period[0] = 50
timer_period[1] = 100

target_dir = 100  # positive to the right
# Every time the prox event is generated, the robot go back accordingly
# of what is sensed on the middle front proximity sensor
LEFT_SENSOR = 0
LEFT_CENTER_SENSOR = 1
FRONT_SENSOR = 2
RIGHT_CENTER_SENSOR = 3
RIGHT_SENSOR = 4

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


def judge_severity():
    global prox_horizontal, danger_state, danger_dir
    danger_state = SAFE
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


def danger_nav():
    global motor_left_target, motor_right_target, toggle0, target_dir

    # Simple cases, one side is totally safe, the other is not
    if (danger_dir[LEFT_SENSOR] == SAFE):
        turn(l)
        return
    elif (danger_dir[RIGHT_SENSOR] == SAFE):
        turn(r)
        return
    # Consider target direction, check if it safe to go there
    # If not, enter backwards mode
    if (target_dir > 0):
        if (danger_dir[RIGHT_SENSOR] < STOP):
            turn(r)
        else:
            turn(b)
    else:
        if (danger_dir[LEFT_SENSOR] < STOP):
            turn(l)
        else:
            turn(b)
    # Just here for sanity
    turn(b)


def turn(dir):
    global motor_right_target, motor_left_target
    if dir == r:
        motor_right_target = -500
        motor_left_target = 500
    elif dir == l:
        motor_right_target = 500
        motor_left_target = -500
    else:
        motor_right_target = -300
        motor_left_target = -300


def potential_field():
    global prox_horizontal, motor_left_target, motor_right_target
    x = prox_horizontal
    y1 = 0
    y2 = 0
    for i in range(5):
        y1 += Wl[i] * x[i] // 200
        y2 += Wr[i] * x[i] // 200
    motor_left_target = motor_left_target // 2 + y1
    motor_right_target = motor_right_target // 2 + y2


@onevent
def prox():
    global prox_horizontal, motor_left_target, motor_right_target, leds_top
    if (toggle0 or toggle1):
        return
    judge_severity()
    if (danger_state == STOP):  # Override all other nav, thymio needs to turn to safety
        leds_top = [32, 0, 0]
        danger_nav()

    elif (danger_state == WARN):  # Add potential field vector to whatever nav we used before
        leds_top = [16, 16, 0]
        potential_field()

    else:  # Safe nav, no control needed
        motor_left_target = 100
        motor_right_target = 100
        leds_top = [0, 16, 16]


@onevent
def timer0():
    global toggle0
    if (not toggle0):  # If unasked for, do nothing
        return
    else:
        toggle0 += 1
    if (toggle0 >= 5):
        toggle0 = 0  # De-activation


@onevent
def timer1():
    global toggle1
    if (not toggle1):
        return
    else:
        toggle1 += 1
    if (toggle1 >= 5):
        toggle1 = 0

