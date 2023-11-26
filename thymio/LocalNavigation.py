import asyncio
from tdmclient import ClientAsync, aw

timer_period = [50, 100]

target_dir = 100  # positive to the right

LEFT_SENSOR = 0
LEFT_CENTER_SENSOR = 1
FRONT_SENSOR = 2
RIGHT_CENTER_SENSOR = 3
RIGHT_SENSOR = 4

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

# Initialize the Thymio client
client = ClientAsync()


async def judge_severity():
    global danger_state, danger_dir
    danger_state = SAFE
    for i in range(5):
        if prox_horizontal[i] > STOP_THRESH:
            danger_dir[i] = STOP
            danger_state = STOP
        elif prox_horizontal[i] > WARN_THRESH:
            danger_dir[i] = WARN
            if danger_state < WARN:
                danger_state = WARN
        else:
            danger_dir[i] = SAFE


async def danger_nav():
    global motor_left_target, motor_right_target, toggle0, target_dir

    if danger_dir[LEFT_SENSOR] == SAFE:
        await turn(l)
    elif danger_dir[RIGHT_SENSOR] == SAFE:
        await turn(r)
    elif target_dir > 0:
        if danger_dir[RIGHT_SENSOR] < STOP:
            await turn(r)
        else:
            await turn(b)
    else:
        if danger_dir[LEFT_SENSOR] < STOP:
            await turn(l)
        else:
            await turn(b)


async def turn(dir):
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


async def potential_field():
    global prox_horizontal, motor_left_target, motor_right_target
    x = prox_horizontal
    y1 = 0
    y2 = 0
    for i in range(5):
        y1 += Wl[i] * x[i] // 200
        y2 += Wr[i] * x[i] // 200
    motor_left_target = motor_left_target // 2 + y1
    motor_right_target = motor_right_target // 2 + y2


async def prox_handler():
    global prox_horizontal, motor_left_target, motor_right_target, leds_top
    if toggle0 or toggle1:
        return
    await judge_severity()
    if danger_state == STOP:
        leds_top = [32, 0, 0]
        await danger_nav()
    elif danger_state == WARN:
        leds_top = [16, 16, 0]
        await potential_field()
    else:
        motor_left_target = 100
        motor_right_target = 100
        leds_top = [0, 16, 16]


async def timer0_handler():
    global toggle0
    if not toggle0:
        return
    else:
        toggle0 += 1
    if toggle0 >= 5:
        toggle0 = 0


async def timer1_handler():
    global toggle1
    if not toggle1:
        return
    else:
        toggle1 += 1
    if toggle1 >= 5:
        toggle1 = 0


async def main():
    Client = ClientAsync()
    node = aw(Client.wait_for_node())
    aw(node.lock())

    # load the program onto the thymio
    await node.compile(prog)



# Define the Aseba program
prog = f"""
var timer0_period = {timer_period[0]}
var timer1_period = {timer_period[1]}

onevent prox
  call prox_handler

onevent timer0
  call timer0_handler

onevent timer1
  call timer1_handler
"""

# Run the Aseba program asynchronously

if __name__ == "__main__":
    ClientAsync.run_async_program(main)