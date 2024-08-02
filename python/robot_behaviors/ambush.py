from robot_util import *

# GLOBALS ##################################################################################################
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination, destination_circle = predator.step.location, None
controller_timer = Timer(5.0)   # reset each set_destination if expires robot stops movement
surge_timer = Timer(0.5) # TODO: check this may bot work
controller_kill_switch = 1

current_experiment_name = ""
episode_in_progress = False

possible_destinations = Cell_group()
possible_destinations_weights = []
spawn_locations = Cell_group()
spawn_locations_weights = []

occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
explore_color,  pursue_color, spawn_color = "magenta", "cyan", "green"
destination_circle = display.circle(predator.step.location, 0.01, explore_color)

display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())

time_out = 1.0

# add angle to surge cell dict
ambush_region = cell_size * 3
surge_cell_dict = {32: [19, 49], 286: [257, 257], 253: [233, 231], 269: [268, 290], 173: [172, 195]} # key:ambush_cell, value:surge_cells
def get_angle(current_location, target_location):  # TO DO: gonna need to fix this find like my convert to German angle function
    dx = target_location.x - current_location.x
    dy = target_location.y - current_location.y
    angle_radians = math.atan2(dy, dx)
    return math.degrees(angle_radians)
for ambush_cell_id, surge_cell_id in surge_cell_dict.items():
    surge_cell_dict[ambush_cell_id].append(get_angle(world.cells[ambush_cell_id].location, world.cells[surge_cell_id[0]].location))
# TODO: need to fix assigned angle based on ridiculous coordinate system

#CLASSES&FUNCTIONS #############################################################################################
def on_keypress(event):
    """
    Sets up keyboard intervention based on key presses.
    """
    global running, current_predator_destination, controller_timer, controller_kill_switch
    key_actions = {
        "p": ("pause", controller.pause, 0),
        "r": ("resume", controller.resume, 1),
        "m": ("auto", controller.resume, 1, True)
    }
    action = key_actions.get(event.key)
    if action:
        print(action[0])                    # print string associated with action
        action[1]()                         # change controller instance state
        controller_kill_switch = action[2]    # change controller_kill_switch variable assignment

        if action[3]:
            controller_timer = Timer(5.0)
            current_predator_destination = prey.step.location # TODO: need location assignment
            controller.set_destination(current_predator_destination)
            destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)

# TODO: issue I am only getting prey step when the prey is visible
def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(time_out)
        predator.step = step
    else:
        prey.is_valid = Timer(time_out) # pursue when prey is seen
        prey.step = step
        AmbushManager.prey_state = prey.step.location.dist(world.cells[AmbushManager.current_ambush_cell].location) <= ambush_region
        surge_timer.reset()

def on_capture(frame:int):
    print("PREY CAPTURED")
    controller.set_behavior(0)


def get_experiment_file (experiment_name, current_experiment_name):
    return get_experiment_folder(experiment_name) + current_experiment_name + "_experiment.json"


def on_experiment_started(experiment):
    global current_predator_destination, destination_circle
    print("EXPERIMENT STARTED: ", experiment)
    experiments[experiment.experiment_name] = experiment.copy()

    # select initial ambush cell
    AmbushManager.current_ambush_cell = AmbushManager.select_random_cell(list(surge_cell_dict.keys()))
    print(f"Ambush cell selected: {AmbushManager.current_ambush_cell}")

    # set ambush cell as current destination
    current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location
    current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
    AmbushManager.state = "AMBUSH"
    destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)
    controller.set_destination(current_predator_destination, current_predator_heading)     # set destination
    controller_timer.reset()
    # TODO: add optional ambush cell message so that we send steps when prey in region probably to set behavior ********

def on_episode_finished(m):
    global episode_in_progress, current_predator_destination
    print("EPISODE FINISHED")

    controller.set_behavior(0)
    controller.pause()              # TODO: do I need this
    episode_in_progress = False

    # update ambush cell bias - loop through previous trajectory
    experiment_state = experiment_service.get_experiment(current_experiment_name)
    last_episode_file = get_episode_file(current_experiment_name, experiment_state.episode_count-1)
    last_trajectory = Episode.load_from_file(last_episode_file).trajectories.get_agent_trajectory("prey")
    last_trajectory_np = last_trajectory.get('location').to_numpy_array() # TODO: May need to update cellworld
    AmbushManager.update_bias(last_trajectory_np)

    # select ambush cell for next episode
    AmbushManager.current_ambush_cell = AmbushManager.select_random_cell(list(surge_cell_dict.keys()))
    print(f"Ambush cell selected: {AmbushManager.current_ambush_cell}")

    # send robot to new ambush cell
    controller.resume()
    current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location
    current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
    AmbushManager.state = "AMBUSH"
    controller.set_destination(current_predator_destination, current_predator_heading)     # set destination
    controller_timer.reset()                                                                # reset controller timer
    destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=spawn_color)
    # TODO: add optional ambush cell message so that we send steps when prey in region probably to set behavior ********

def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name
    current_experiment_name = parameters.experiment_name


def on_prey_entered_arena():
    print("PREY ENTERED ARENA")
    global episode_in_progress, controller_timer
    episode_in_progress = True
    controller_timer = Timer(5.0)


class AmbushManager:
    # global display
    current_ambush_cell = None
    bias = {32: 1e-9, 286: 1e-9, 253: 1e-9, 269: 1e-9, 173: 1e-9}
    decay_rate = 0.75
    state = None            # Surge, Ambush, Ambush Reached
    prey_state = 0          # True if last prey step in ambush zone

    @classmethod
    def select_random_cell(cls, ambush_cell_id):

        # select new ambush cell
        total_bias = sum(cls.bias.values())
        if total_bias == 0:
            cls.current_ambush_cell = random.choice(ambush_cell_id)
        else:
            probabilities = [v / total_bias for v in cls.bias.values()]
            cls.current_ambush_cell = random.choices(list(cls.bias.keys()), probabilities)[0]

        return cls.current_ambush_cell

    @classmethod
    def update_bias(cls, prey_trajectory: np.array):
        # Decay the scores for each ambush cell
        for ambush_cell_id in cls.bias.keys():
            cls.bias[ambush_cell_id] *= cls.decay_rate

        # Track entrance and exit state for each ambush zone
        entered_cells = set()
        for x, y in prey_trajectory:
            current_position = Location(x, y)
            for ambush_cell_id in cls.bias.keys():
                cell_location = world.cells[ambush_cell_id].location
                if current_position.dist(cell_location) <= cell_size * 3:
                    if ambush_cell_id not in entered_cells:
                        # The agent has entered a new ambush zone
                        entered_cells.add(ambush_cell_id)
                        cls.bias[ambush_cell_id] += 1.0
                else:
                    # The agent has exited the ambush zone remove it from set
                    if ambush_cell_id in entered_cells:
                        entered_cells.remove(ambush_cell_id)

        # special rule for ambush cell with cell id = 32
        values = list(cls.bias.values())
        max_value, second_max_value = sorted(set(values), reverse=True)[:2]   # TODO: play around with decay and saturation limit
        if cls.bias[32] == max_value:
            cls.bias[32] = second_max_value

        # saturation limit
        for ambush_cell_id in cls.bias.keys():
            if cls.bias[ambush_cell_id] > 10:
                cls.bias[ambush_cell_id] = 10

        # plot new bias for ambush cells
        cmap = plt.cm.Reds([weight / max(cls.bias.values()) for weight in cls.bias.values()])
        for i, ambush_cell_id in enumerate(cls.bias.keys()):
            display.cell(cell= world.cells[ambush_cell_id], color=cmap[i])




# SETUP ###########################################################################################################

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
experiment_service.subscribe()
experiments = {}


# CONNECT TO CONTROLLER
controller = ControllerClient()
if not controller.connect("127.0.0.1", 4590):
    print("failed to connect to the controller")
    exit(1)
controller.set_request_time_out(10000)
controller.subscribe()
controller.on_step = on_step

# set destination test
# robot_location = world.cells[127].location
# controller.set_destination(robot_location)  # resend destination

# MAIN ##############################################################################################################
running = True
while running:
    # IF PAUSE DONT EXECUTE REST OF LOOP ROBOT STOPS MOVING
    if not controller_kill_switch:
        controller.pause()
        continue

    # IF MOUSE IN AMBUSH REGION SET DESTINATION TO CORRECT SURGE CELL
    # TODO: might have to add like an ambush reached stipulation depends on how I want surge to look
    if AmbushManager.prey_state and surge_timer: #and AmbushManager.state == "AMBUSH_REACHED":
        print("SURGE PREY IN AMBUSH REGION")
        if world.cells[AmbushManager.current_ambush_cell].location.y > prey.step.location.y:
            surge_location = world.cells[surge_cell_dict[AmbushManager.current_ambush_cell][1]].location
        else:
            surge_location = world.cells[surge_cell_dict[AmbushManager.current_ambush_cell][0]].location
        current_predator_destination = surge_location
        AmbushManager.ambush_state = "SURGE"
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=pursue_color)
    # ELSE SET DESTINATION TO AMBUSH CELL
    elif (AmbushManager.state == "SURGE" or not surge_timer) and (AmbushManager.current_ambush_cell != None):
        print("DRIVE BACK TO AMBUSH CELL")
        current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location
        current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
        AmbushManager.state = "AMBUSH"
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=pursue_color)

    # CHECK IF CURRENT DESTINATION REACHED PAUSE ROBOT
    if current_predator_destination.dist(predator.step.location) < (cell_size * 1):
        print("DESTINATION REACHED")
        if AmbushManager.state == "AMBUSH":
            AmbushManager.state = "AMBUSH_REACHED"
        controller.pause()
    # IF NOT SEND ROBOT TO DESTINATION
    else:
        print(f"CURRENT BEHAVIOR STATE: {AmbushManager.state}")
        controller.pause()
        if AmbushManager.state == "AMBUSH":
            controller.set_destination(current_predator_destination, surge_cell_dict[AmbushManager.current_ambush_cell][2])     # set destination
        else:
            controller.set_destination(current_predator_destination)
        controller_timer.reset()
        controller.resume()

    # PLOT AGENT STATES
    if prey.is_valid:
        display.agent(step=prey.step, color="green", size=10)
    else:
        display.agent(step=prey.step, color="gray", size=10)
    if predator.is_valid:
        display.agent(step=predator.step, color="blue", size=10)
    else:
        display.agent(step=predator.step, color="gray", size=10)
    display.fig.canvas.draw_idle()
    display.fig.canvas.start_event_loop(0.001)
    sleep(0.1)

controller.unsubscribe()
controller.stop()