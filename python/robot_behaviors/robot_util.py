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


# KEEPER INTERCEPT FUNCTIONS
def distance_to_intercept_point(route: np.array, start_index: int = 0, end_index: int = -1) -> float:
    """Calculate distance between two specified points on a given path."""
    if end_index < 0:
        end_index = len(route) - 1

    if start_index <= end_index:
        distances = np.sqrt(np.sum(np.diff(route, axis=0) ** 2, axis=1))
        total_distance = np.sum(distances[start_index:end_index])
        return total_distance
    else:
        print("Start index greater than end index on route")
        return 0.0


def closest_open_cell(current_id: int, robot_world, world_cells, world_free_cells) -> int:
    """Find the closest open cell if the current cell is occluded."""
    if robot_world.cells[current_id].occluded:
        current_location = np.array([robot_world.cells[current_id].location.x, robot_world.cells[current_id].location.y])
        distances = np.linalg.norm(world_free_cells - current_location, axis=1)
        closest_index_in_free = np.argmin(distances)
        closest_free_cell = world_free_cells[closest_index_in_free]
        closest_index_in_cells = np.where((world_cells == closest_free_cell).all(axis=1))[0][0]
        return closest_index_in_cells
    return current_id


def get_robot_interception_path(start_cell_location: cellworld.Location, end_cell_id: int, robot_world, path_object, robot_world_cells, robot_world_free_cells):
    """Get robot path from a start cell to end cell"""
    start_cell_id = closest_open_cell(robot_world.cells.find(start_cell_location), robot_world, robot_world_cells, robot_world_free_cells)
    end_cell_id = closest_open_cell(end_cell_id, robot_world, robot_world_cells, robot_world_free_cells)
    return path_object.get_path(robot_world.cells[start_cell_id], robot_world.cells[end_cell_id]).get('location').to_numpy_array()

def estimate_heading(position_window: np.array, num_points_avg=3) -> np.array:
    """Estimate heading direction using averaged start and end points."""
    if len(position_window) < 2 * num_points_avg:
        return np.array([0, 0])

    start_avg = np.mean(position_window[:num_points_avg], axis=0)
    end_avg = np.mean(position_window[-num_points_avg:], axis=0)
    direction_vector = end_avg - start_avg
    norm = np.linalg.norm(direction_vector)

    return direction_vector / norm if norm != 0 else np.array([0, 0])


def project_points_onto_segments(mouse_position, highway_points):
    """Project mouse position onto the segments of the highway."""
    v = highway_points[:-1]  # Start of each segment
    w = highway_points[1:]   # End of each segment
    vw = w - v
    vw_length_squared = np.sum(vw**2, axis=1)
    vw_length_squared[vw_length_squared == 0] = 1  # Prevent division by zero for degenerate segments

    vp = mouse_position - v
    t = np.sum(vp * vw, axis=1) / vw_length_squared
    t = np.clip(t, 0, 1)  # Ensure t is within [0, 1]

    projection = v + t[:, np.newaxis] * vw
    distances = np.linalg.norm(projection - mouse_position, axis=1)

    return projection, distances



def is_heading_towards_highway(mouse_position, highway_points, heading_vector, angle_threshold= 2 *np.pi ): # TODO: change back to np.pi/4
    """Check if the mouse is heading towards a highway."""
    projection_points, distances = project_points_onto_segments(mouse_position, highway_points)
    min_index = np.argmin(distances)
    closest_point = projection_points[min_index]

    tangent_vector = highway_points[min_index + 1] - highway_points[min_index]  # Tangent vector of the closest segment
    tangent_vector /= np.linalg.norm(tangent_vector)  # Normalize the tangent vector

    dot_product = np.dot(heading_vector, tangent_vector)
    angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # A * B = |A||B| cos(theta)
    print(f"Heading Error: {to_degrees(angle)}, Heading Passes: {angle < angle_threshold}")
    return angle < angle_threshold, closest_point


def find_closest_point(current_location, route):
    """Find the closest point on route to the current location."""
    x, y = current_location[0], current_location[1]
    distances = np.sqrt((route[:, 0] - x) ** 2 + (route[:, 1] - y) ** 2)
    min_index = np.argmin(distances)
    # closest_point = route[min_index]
    return min_index, distances[min_index]

def mouse_is_near_and_heading_towards_highway(mouse_position, highway_route, heading_vector, threshold_distance) -> tuple:
    """Check if mouse is near and heading towards a highway."""
    min_index, min_distance = find_closest_point(mouse_position, highway_route)
    is_close = min_distance < threshold_distance
    print(f"Distance Error: {min_distance}, Distance Passes: {is_close}")
    heading_towards, closest_point = is_heading_towards_highway(mouse_position, highway_route, heading_vector)
    return is_close and heading_towards, min_index


def calculate_intercept_time(prey_distance: float, predator_distance: float, PREY_SPEED = 0.75, PREDATOR_SPEED = 0.23) -> bool:
    """
    Calculate if the predator can intercept the prey.
    Compare the time it would take for both predator and prey to reach the intercept point.
    """
    prey_time = prey_distance / PREY_SPEED
    predator_time = predator_distance / PREDATOR_SPEED

    # print(f"Prey time to intercept: {prey_time}, Predator time to intercept: {predator_time}")
    return predator_time < prey_time


def get_cell_route(route, world):
    """Get more coarse highway of cell path for robot path planning"""
    cell_route = []
    for location in route:
        location_cw = Location(location[0], location[1])
        cell_id = world.cells.find(location_cw)
        cell_route.append(cell_id)
    return cell_route


def get_potential_intercept_point_highway_indices(cell_route, highway, world):
    """ Translate cell route to highway indices """
    highway_indices = []
    for id in cell_route:
        coarse_location = world.cells[id].location
        query_point = [coarse_location.x, coarse_location.y]
        distances = np.linalg.norm(highway - query_point, axis=1)
        min_index = np.argmin(distances)
        highway_indices.append(min_index)
    return highway_indices


def backtrack_trajectory(current_index, steps_back, predator_mode):
    print(f"current predator_mode_int {current_index}")
    # Ensure the step size does not go beyond the start of the trajectory
    target_index = current_index - steps_back

    if target_index < 5:
        return -1, "STEALTH_SEARCH"

    # Return the index of the target waypoint
    return target_index, predator_mode


def get_closest_search_key(predator_location, search_cell_dict, world):
    closest_key = None
    closest_key_dist = float('inf')  # Use infinity to ensure any distance is smaller
    for key in search_cell_dict.keys():
        key_dist = predator_location.dist(world.cells[key].location)
        if key_dist < closest_key_dist:
            closest_key_dist = key_dist
            closest_key = key
    return closest_key


def remove_duplicates_preserve_order(lst):
    """Remove duplicates from list while preserving order."""
    seen = set()
    return [x for x in lst if not (x in seen or seen.add(x))]