from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import pickle

df = pd.DataFrame(columns=["Episode", "Type", "Data"])
def fix_coordinate_system(ang):
    ang = (ang - 90) * -1
    while ang < 0:
        ang = ang + 360
    while ang > 360:
        ang = ang - 360
    return ang


def get_angle(current_location, target_location):
    dx = target_location.x - current_location.x
    dy = target_location.y - current_location.y
    angle_radians = math.atan2(dy, dx)
    # return fix_coordinate_system(math.degrees(angle_radians))
    return fix_coordinate_system(round(math.degrees(angle_radians), 0))


def log_data(pickle_file_path, episode, entry_type, data, df):
    # TO USE: df = log_data(pickle_file_path, i, "ambush_cell_id", 1, df)
    new_entry_df = pd.DataFrame([[episode, entry_type, data]], columns=["Episode", "Type", "Data"])
    df = pd.concat([df, new_entry_df], ignore_index=True)
    with open(pickle_file_path, 'wb') as f:
        pickle.dump(df, f)
    return df


