import matplotlib.pyplot as plt
import sys
from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from random import choice, choices
from time import sleep
from json_cpp import JsonList


display = None
episode_in_progress = False
capture_state = False
experiment_log_folder = "/research/data"
current_experiment_name = ""

pheromone_charge = .25
pheromone_decay = 1.0
pheromone_max = 50

possible_destinations = Cell_group()
possible_destinations_weights = []

spawn_locations = Cell_group()
spawn_locations_weights = []

is_spawn = []

new_experiment = False

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


def go_to_start_location():
    global current_predator_destination, destination_circle
    controller.resume()
    current_predator_destination = choice(spawn_locations.get("location"))
    destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = explore_color)
    controller.set_destination(current_predator_destination)  # resend destination
    controller_timer.reset()


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
    global episode_in_progress, capture_state
    print("EPISODE FINISHED")
    current_predator_destination = hidden_location()
    controller.set_destination(current_predator_destination)     # set destination
    destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = spawn_color)
    episode_in_progress = False
    capture_state = False


def on_capture(frame:int):
    global inertia_buffer, capture_state
    capture_state = True
    controller.set_behavior(0)
    inertia_buffer = 1
    print ("PREY CAPTURED")


def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name, current_predator_destination
    current_experiment_name = parameters.experiment_name
    print("New Episode: ", parameters.experiment_name)
    print("Occlusions: ", experiments[parameters.experiment_name].world.occlusions)


def on_prey_entered_arena():
    print("Prey Entered")
    global episode_in_progress, controller_timer, capture_state
    controller.arm()
    episode_in_progress = True
    capture_state = False
    controller_timer = Timer(5.0)


def load_world():
    """
    Load world to display
    """
    global display, world, possible_destinations, possible_destinations_weights, spawn_locations, spawn_locations_weights, is_spawn

    occlusion = Cell_group_builder.get_from_name("hexagonal", occlusions + ".occlusions")
    possible_destinations = world.create_cell_group(Cell_group_builder.get_from_name("hexagonal", occlusions + ".predator_destinations"))
    possible_destinations_weights = [1.0 for x in possible_destinations]
    spawn_locations = world.create_cell_group(Cell_group_builder.get_from_name("hexagonal", occlusions + ".spawn_locations"))
    spawn_locations_weights = [1.0 for x in spawn_locations]
    is_spawn = [len(spawn_locations.where("id", c.id)) > 0 for c in possible_destinations]
    world.set_occlusions(occlusion)
    display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)


def random_location():
    """
    Returns random open location (keep this for cases where there are no hidden locations)
    """
    return choice(world.cells.free_cells().get("location"))


def hidden_location():
    """
    Returns random hidden location in robot_world
    """
    try:    # find random hidden cell
        #new_cell = choices(hidden_cells)
        new_cell = choices(possible_destinations, weights=possible_destinations_weights)[0] # use bias now ??
        new_cell_location = new_cell.location
    except:  # if no hidden locations
        new_cell_location = random_location()
    return new_cell_location


def on_step(step: Step):
    """
    Updates steps and predator behavior
    """
    global behavior

    if step.agent_name == "predator":
        predator.is_valid = Timer(time_out)
        predator.step = step
        #display.circle(step.location, 0.002, "royalblue")    # plot predator path (steps)
        if behavior != ControllerClient.Behavior.Explore:
            controller.set_behavior(ControllerClient.Behavior.Explore) # explore when prey not seen
            behavior = ControllerClient.Behavior.Explore
    else:
        prey.is_valid = Timer(time_out) # pursue when prey is seen
        prey.step = step
        # controller.set_behavior(ControllerClient.Behavior.Pursue)


def on_click(event):
    """
    Assign destination by clicking on map
    Right-click to start experiment
    """
    global current_predator_destination

    if event.button == 1:
        controller.resume()
        location = Location(event.xdata, event.ydata)
        location = Location(event.xdata, event.ydata)
        cell_id = world.cells.find(location)
        destination_cell = world.cells[cell_id]
        if destination_cell.occluded:
            print("can't navigate to an occluded cell")
            return
        current_predator_destination = destination_cell.location
        controller.set_destination(destination_cell.location)
        destination_circle.set(center = (current_predator_destination.x, current_predator_destination.y), color = explore_color)
    # else:
    #     print("starting experiment")
    #     exp = experiment_service.start_experiment(                  # call start experiment
    #         prefix="PREFIX",
    #         suffix="SUFFIX",
    #         occlusions=occlusions,
    #         world_implementation="canonical",
    #         world_configuration="hexagonal",
    #         subject_name="SUBJECT",
    #         duration=10)
    #     print("Experiment Name: ", exp.experiment_name)
    #     r = experiment_service.start_episode(exp.experiment_name)   # call start episode
    #     print(r)


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

random_spawn = False
if len(sys.argv) == 2:
    occlusions = sys.argv[1]
elif len(sys.argv) == 3:
    random_spawn = sys.argv[2]
else:
    print('Needs 1 or 2 arguments')

inertia_buffer = 1
time_out = 1.0      # step timer for predator and preyQ
NOLOCATION = Location(-1000, -1000)

robot_visibility = None
controller_state = 1 # resume = 1, pause = 0
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

if not experiment_service.connect("127.0.0.1"):
    print("Failed to connect to experiment service")
    exit(1)
experiment_service.set_request_time_out(5000)
experiment_service.subscribe()                  # having issues subscribing to exp service

experiments = {}


# CONNECT TO CONTROLLER
controller_timer = Timer(5.0)    # initialize controller timer variable
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


# ADD PREDATOR DESTINATION TO WORLD - initialize at current predator location
explore_color = "magenta"
pursue_color = "cyan"
spawn_color = "green"
destination_circle = display.circle(predator.step.location, 0.01, explore_color)


running = True
while running:

    if current_predator_destination.dist(predator.step.location) < (cell_size * inertia_buffer):
        controller.pause()
        current_predator_destination = predator.step.location  # assign destination to current predator location (artificially reach goal when "close enough")
    elif not controller_timer:
        controller.set_destination(current_predator_destination)  # resend destination
        controller_timer.reset()
    elif capture_state & episode_in_progress:
        current_predator_destination = NOLOCATION
        controller.pause()
        controller.set_behavior(0)
        controller.set_destination(current_predator_destination)
        controller_timer.reset()

    # plotting the current location of the predator and prey
    if prey.is_valid:
        display.agent(step=prey.step, color="green", size=10)

    else:
        display.agent(step=prey.step, color="gray", size=10)

    if predator.is_valid:
        if capture_state:
            display.agent(step=predator.step, color="red", size=10)
        else:
            display.agent(step=predator.step, color="blue", size=10)

    else:
        display.agent(step=predator.step, color="gray", size=10)

    # display.update()
    display.fig.canvas.draw_idle()
    display.fig.canvas.start_event_loop(0.001)
    sleep(0.1)

controller.unsubscribe()
controller.stop()

