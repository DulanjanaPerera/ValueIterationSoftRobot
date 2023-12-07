from Tetra_pybullet import TetraSim
import time
import keyboard
from Rolling_gait import Rolling
import json
import os


def DoAction(gait, env, head: str, tail: str) -> None:
    if head == 'r':
        if tail == 'b':
            gait.red_top("blue")
        elif tail == 'g':
            gait.red_top("green")
        elif tail == 'y':
            gait.red_top("yellow")
    elif head == 'b':
        if tail == 'r':
            gait.blue_top("red")
        elif tail == 'g':
            gait.blue_top("green")
        elif tail == 'y':
            gait.blue_top("yellow")
    elif head == 'g':
        if tail == 'r':
            gait.green_top("red")
        elif tail == 'b':
            gait.green_top("blue")
        elif tail == 'y':
            gait.green_top("yellow")
    elif head == 'y':
        if tail == 'r':
            gait.yellow_top("red")
        elif tail == 'b':
            gait.yellow_top("blue")
        elif tail == 'g':
            gait.yellow_top("green")
    action = gait.action

    i = 0
    count = 0
    flag_stop = 0
    a = [0, 0, 0, 0, 0, 0, 0, 0]
    while 1:
        if (count > 120) and (count % 36 == 0) and flag_stop == 0:
            a = action[i]
            i += 1

        if i == len(action):
            i = 0
            flag_stop = 1
            break

        env.step(a)

        time.sleep(1. / 240)
        count += 1
    fcount = count
    while count < fcount + 250:
        env.step(a)
        time.sleep(1. / 240)
        count += 1

#load the policy
current_directory = os.getcwd()
file_path = os.path.join(current_directory, "Data", "policy_2step.json")

with open(file_path, "r") as file:
    policy = json.load(file, parse_float=float)

gui = True
tetra = TetraSim(gui, 190)
print("----Object created")
tetra.run()
print("----Simulation runs")
gait = Rolling()
gait.half_circle_coordinates(0.09)
print("----Gaits are generated")
H = 2

state = [-0.2, 0.0, 2]

h = 1
count = 0
action = [[0, 0, 0, 0, 0, 0, 0, 0]]
a = action[0]
ind = 0
flag_stop = 0
flag_run = 0
while 1:
    if (count > 120) and (count % 36 == 0) and flag_stop == 0:
        a = action[ind]
        ind += 1
        if ind == len(action):
            ind = 0
            flag_stop = 1

    if keyboard.is_pressed('a'):
        tetra.setEnv(pos=[state[0], state[1], 0.5], ori=tetra.o_states[state[2]])
        print("start reading the policy")
        flag_run = 1

    if flag_run == 1 and h <= H:
        print("step: " + str(h))
        [head, tail] = policy[str(h)][str(state[0])][str(state[1])][str(state[2])]
        print("Head: " + head + "    tail: " + tail)
        DoAction(gait, tetra, head, tail)
        [px, py, pz] = list(tetra.getRobotBasePosition())
        ori = tetra.OrinetationIndex()
        state = [px, py, ori]
        print("States: " + str(state) + "\n\n")
        h += 1

    if count == 240 * 600 or keyboard.is_pressed('q'):
        print("Quit the simulation")
        tetra.endSimulation()
        break

    ori = tetra.step(a)

    if gui:
        time.sleep(1. / 240)
        count += 1
