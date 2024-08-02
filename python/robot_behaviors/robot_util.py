from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from time import sleep
import matplotlib.pyplot as plt
import numpy as np

########### CONSTANTS ############
experiment_log_folder = "/research/data"


###############################


class AgentData:
    def __init__(self, agent_name: str):
        self.is_valid = None # timers for predator and prey updates
        self.step = Step()
        self.step.agent_name = agent_name


def get_experiment_folder (experiment_name):
    return experiment_log_folder + "/" + experiment_name.split('_')[0] + "/" + experiment_name

def get_episode_folder (experiment_name, episode_number):
    return get_experiment_folder(experiment_name) + f"/episode_{episode_number:03}"


def get_episode_file (experiment_name, episode_number):
    return get_episode_folder(experiment_name, episode_number) + f"/{experiment_name}_episode_{episode_number:03}.json"


def on_keypress(event):
    """
    Sets up keyboard intervention
    """
    global running, current_predator_destination, controller_timer, controller_state

    if event.key == "p":
        print("pause")
        controller.pause()
        controller_state = 0
    if event.key == "r":
        print("resume")
        controller.resume()
        controller_state = 1
    if event.key == "q":
        print("quit")
        controller.pause()
        running = False
    if event.key == "m":
        print("auto")
        controller_state = 1
        controller.resume()                                     # change controller state to Playing
        controller_timer = Timer(5.0)                           # set initial destination and timer
        current_predator_destination = hidden_location()        # assign new destination
        controller.set_destination(current_predator_destination)
        destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = explore_color)
