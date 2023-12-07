from Tetra_pybullet import TetraSim
import time
import os
from Rolling_gait import Rolling
from ValueIteration import ValueIteration
import json

"""
str - 0-0.39, dir - 0-3.14
front-blue, right-yellow, left-green, top-red
"""

current_directory = os.getcwd()

file_pathP = os.path.join(current_directory, "Data", "policy_2step.json")

# render the simulation
gui = True
tetra = TetraSim(gui, 190)
tetra.run()
gait = Rolling()
gait.half_circle_coordinates(0.09)

start = [0.0, 0.0, 2]
goal = [1.2, 0.0, 1]

vi = ValueIteration(tetra, gait, 2, start, goal)
start_time = time.time()
vi.vi()
end_time = time.time()

elapsed_time = end_time - start_time

print(f"Elapsed Time: {elapsed_time} seconds")


# save the policy at each step. This will become a very big file ~700MB
policyStep = vi.stepPolicy
policyStep_j = json.dumps(policyStep, indent=4)

with open(file_pathP, "w") as file:
    file.write(policyStep_j)

