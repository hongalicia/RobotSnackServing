import csv
import numpy as np
from enum import Enum
import os

class Mode(Enum):
    MOVE = 1
    OPEN = 2
    HALF_OPEN = 3
    CLOSE = 4
    CLOSE_TIGHT = 5


status_open = [0, 0, 0]
status_close = [1, 0, 0]
status_half_open = [0, 1, 0]
status_close_tight = [1, 1, 0]


class Movement:
    def __init__(self, mode, joint_value=None):
        self.mode = mode
        self.joint_value = joint_value          
        self.joints_values = []                 


TRAJECTORY_CACHE = {}

def parse_trajectory_rows(rows):
    movements = []
    index = 0
    gripper_prev = None

    for row in rows:
        if index == 0:
            index += 1
            continue

        joint_values = (row[0][1:len(row[0]) - 1]).split(', ')
        joint_values_float = []

        for joint in joint_values:
            joint_values_float.append(float(joint))

        if joint_values_float[6:9] == status_open:
            if gripper_prev is None or gripper_prev != status_open:
                move = Movement(Mode.OPEN)
            else:
                move = Movement(Mode.MOVE, joint_values_float[0:6])

        elif joint_values_float[6:9] == status_close:
            if gripper_prev is None or gripper_prev != status_close:
                move = Movement(Mode.CLOSE)
            else:
                move = Movement(Mode.MOVE, joint_values_float[0:6])

        elif joint_values_float[6:9] == status_half_open:
            if gripper_prev is None or gripper_prev != status_half_open:
                move = Movement(Mode.HALF_OPEN)
            else:
                move = Movement(Mode.MOVE, joint_values_float[0:6])

        elif joint_values_float[6:9] == status_close_tight:
            if gripper_prev is None or gripper_prev != status_close_tight:
                move = Movement(Mode.CLOSE_TIGHT)
            else:
                move = Movement(Mode.MOVE, joint_values_float[0:6])

        else:
            move = Movement(Mode.MOVE, joint_values_float[0:6])

        movements.append(move)
        gripper_prev = joint_values_float[6:9]

    return movements


def normalize_traj_name(name: str) -> str:
    base = os.path.basename(name)        # close_2nd_lid.csv
    stem, _ = os.path.splitext(base)     # close_2nd_lid
    return stem

def get_trajectory(name):
    key = normalize_traj_name(name)
    if key not in TRAJECTORY_CACHE:
        raise KeyError(f"Trajectory '{key}' not found in cache.\n"
                       f"Available: {list(TRAJECTORY_CACHE.keys())}")
    return TRAJECTORY_CACHE[key]