import cellworld
from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from time import sleep
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import pickle
import itertools

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


class AgentData:
    def __init__(self, agent_name: str):
        self.is_valid = None  # timers for predator and prey updates
        self.step = Step()
        self.step.agent_name = agent_name


def select_random_cell(cell_group_ids: int, previous_destination: cellworld.Location, min_distance : int, world: cellworld.World) -> cellworld.Location:
    """ Selects a random cell from a provided group of cells that is a specified distance from the last selected cell
    """
    eligible_cells = [cell_id for cell_id in cell_group_ids if world.cells[cell_id].location.dist(previous_destination) >= (min_distance * world.implementation.cell_transformation.size)]

    if not eligible_cells:
        print("No eligible cells found that meet the distance requirement.")
        world.cells[random.choice(cell_group_ids)].location

    return world.cells[random.choice(eligible_cells)].location

def generate_pattern(start):
    # directions = ['north', 'middle', 'south']
    # pattern = []
    if start == 'north':
        pattern = ['middle', 'south', 'middle', 'north']
    elif start == 'middle':                                     # only time it would be middle is if spawned middle - robot will start north
        pattern = ['north', 'middle', 'south', 'middle']
    else:
        pattern = ['middle', 'north', 'middle', 'south']
    return itertools.cycle(pattern)


def get_patrol_side_waypoint(waypoint_id, mouse_side, patrol_path, ep_manager_mode):
    assert (mouse_side == "north" or mouse_side == "south") # the mouse should be valid and detected on a side if this function is called
    key = next((k for k,v in patrol_path.items() if v == waypoint_id), 'middle') # current waypoint id to cardinal direction string

    if ep_manager_mode == "patrol":
        # if mouse headed to waypoint not in strategic patrol path
        if key != mouse_side and key != 'middle': # todo: what should i do if waypoint is opposite side
            return patrol_path['middle']
        # otherwise continue current heading then reassign
        else:
            return waypoint_id
    else:
        # catches instances where mouse seen on opposite side of habitat
        if key != mouse_side and key != 'middle':  # if waypoint is opposite to side go middle
            return patrol_path['middle']
        elif key != 'middle':
            return patrol_path['middle']
        else:
            return patrol_path[mouse_side]

def get_patrol_side_waypoint_old(value_to_find, mouse_side, patrol_path):
    assert (mouse_side == "north" or mouse_side == "south")
    key = [k for k, v in patrol_path.items() if v == value_to_find][0]
    if key!= 'middle':
        return patrol_path['middle']
    else:
        return patrol_path[mouse_side]