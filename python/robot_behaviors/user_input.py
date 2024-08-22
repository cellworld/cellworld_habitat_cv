import sys
from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from random import choice, choices
from time import sleep


display = None
episode_in_progress = False
experiment_log_folder = "/research/data"
current_experiment_name = ""


class AgentData:
    def __init__(self, agent_name: str):
        self.is_valid = None # timers for predator and prey updates
        self.step = Step()
        self.step.agent_name = agent_name

def get_experiment_folder (experiment_name):
    return experiment_log_folder + "/" + experiment_name.split('_')[0] + "/" + experiment_name


def get_experiment_file (experiment_name):
    return get_experiment_folder(experiment_name) + current_experiment_name + "_experiment.json"

def get_episode_folder (experiment_name, episode_number):
    return get_experiment_folder(experiment_name) + f"/episode_{episode_number:03}"


def get_episode_file (experiment_name, episode_number):
    return get_episode_folder(experiment_name, episode_number) + f"/{experiment_name}_episode_{episode_number:03}.json"


def on_experiment_started(experiment):
    """
    To start experiment right click on map
    """
    global current_predator_destination, destination_circle, controller_timer
    print("Experiment started:", experiment)
    experiments[experiment.experiment_name] = experiment.copy()
    current_predator_destination = predator.step.location
    destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = explore_color)


def on_episode_finished(m):
    print("EPISODE FINISHED")
    current_predator_destination = predator.step.location
    controller.set_destination(current_predator_destination)     # set destination
    destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = spawn_color)
    controller.pause()

def on_capture( frame:int ):
    global inertia_buffer
    controller.set_behavior(0)
    inertia_buffer = 1


def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name
    current_experiment_name = parameters.experiment_name
    print("New Episode: ", parameters.experiment_name)
    print("Occlusions: ", experiments[parameters.experiment_name].world.occlusions)


def on_prey_entered_arena():
    print("Prey Entered")

def load_world():
    """
    Load world to display
    """
    global display, world, possible_destinations, possible_destinations_weights, spawn_locations, spawn_locations_weights, is_spawn

    occlusion = Cell_group_builder.get_from_name("hexagonal", occlusions + ".occlusions")
    world.set_occlusions(occlusion)
    display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)

    spawn_locations = world.create_cell_group(Cell_group_builder.get_from_name("hexagonal", occlusions + ".spawn_locations"))
    for spawn_cell in spawn_locations:
        display.cell(cell=spawn_cell, color='grey', alpha = 0.5)



def on_step(step: Step):
    """
    Updates steps and predator behavior
    """
    global behavior

    if step.agent_name == "predator":
        predator.is_valid = Timer(time_out)
        predator.step = step
        if behavior != ControllerClient.Behavior.Explore:
            controller.set_behavior(ControllerClient.Behavior.Explore) # explore when prey not seen
            behavior = ControllerClient.Behavior.Explore
    else:
        prey.is_valid = Timer(time_out) # pursue when prey is seen
        prey.step = step
        controller.set_behavior(ControllerClient.Behavior.Pursue)


def on_click(event):
    """
    Select predator destinations
    """
    global current_predator_destination

    if event.button == 1:
        controller.resume()
        location = Location(event.xdata, event.ydata)
        cell_id = world.cells.find(location)
        destination_cell = world.cells[cell_id]
        if destination_cell.occluded:
            print("can't navigate to an occluded cell")
            return
        current_predator_destination = destination_cell.location
        controller.set_destination(destination_cell.location)
        destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = explore_color)



def on_keypress(event):
    """
    Sets up keyboard intervention
    """
    global running, current_predator_destination, controller_timer, controller_state

    if event.key == "p":
        print("pause")
        controller.pause()
    if event.key == "r":
        print("resume")
        controller.resume()
    if event.key == "q":
        print("quit")
        controller.pause()
        running = False



occlusions = sys.argv[1]
inertia_buffer = 1
time_out = 1.0      # step timer for predator and prey
robot_visibility = None

# create world
world = World.get_from_parameters_names("hexagonal", "canonical")
load_world()
cell_size = world.implementation.cell_transformation.size

#  create predator and prey objects
predator = AgentData("predator")
prey = AgentData("prey")

# set initial destination and behavior
current_predator_destination = predator.step.location  # initial predator destination
behavior = -1                                          # Explore or Pursue


# CONNECT TO EXPERIMENT SERVER
experiment_service = ExperimentClient()
experiment_service.on_experiment_started = on_experiment_started
experiment_service.on_episode_started = on_episode_started
experiment_service.on_prey_entered_arena = on_prey_entered_arena
experiment_service.on_episode_finished = on_episode_finished
experiment_service.on_capture = on_capture

# connect to experiment server
if not experiment_service.connect("127.0.0.1"):
    print("Failed to connect to experiment service")
    exit(1)
experiment_service.set_request_time_out(5000)
experiment_service.subscribe()
experiments = {}

# connect to controller server
controller_timer = Timer(5.0)
controller = ControllerClient()
if not controller.connect("127.0.0.1", 4590):
    print("failed to connect to the controller")
    exit(1)
controller.set_request_time_out(10000)
controller.subscribe()
controller.on_step = on_step

# INITIALIZE KEYBOARD & CLICK INTERRUPTS
cid1 = display.fig.canvas.mpl_connect('button_press_event', on_click)
cid_keypress = display.fig.canvas.mpl_connect('key_press_event', on_keypress)

# ADD PREDATOR AND PREY TO WORLD
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())

# ADD PREDATOR DESTINATION TO WORLD
explore_color = "magenta"
spawn_color = 'cyan'
destination_circle = display.circle(predator.step.location, 0.01, explore_color) # initialize at current predator location

running = True
while running:
    # if predator close enough to set desitination it will stop moving
    if current_predator_destination.dist(predator.step.location) < (cell_size * inertia_buffer):
        controller.pause()
        current_predator_destination = predator.step.location  # assign destination to current predator location (artificially reach goal when "close enough")
    elif not controller_timer:
        controller.set_destination(current_predator_destination)  # resend destination
        controller_timer.reset()

    # plotting the current location of the predator and prey
    if prey.is_valid:
        display.agent(step=prey.step, color="green", size=10)

    else:
        display.agent(step=prey.step, color="gray", size=10)

    if predator.is_valid:
        display.agent(step=predator.step, color="blue", size=10)

    else:
        display.agent(step=predator.step, color="gray", size=10)

    # display.update()
    display.fig.canvas.draw_idle()
    display.fig.canvas.start_event_loop(0.001)
    sleep(0.1)

controller.unsubscribe()
controller.stop()

