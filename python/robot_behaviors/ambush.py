from robot_util import *

# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
explore_color = "cyan"
ambush_region = cell_size * 3

# UTIL SETUP
surge_cell_dict = {32: [19, 49], 277: [257, 257], 253: [233, 231], 269: [268, 290], 173: [172, 195]}    # key:ambush_cell, value:surge_cells
for ambush_cell_id, surge_cell_id in surge_cell_dict.items():
    surge_cell_dict[ambush_cell_id].append(get_angle(world.cells[ambush_cell_id].location, world.cells[surge_cell_id[0]].location))
print(f'Surge Cell Dictionary: {surge_cell_dict}')
episode_count = 0
df = pd.DataFrame()

# EXPERIMENT/CONTROLLER SETUP
experiment_log_folder = "/research/data"
episode_in_progress = False
controller_timer = None
current_predator_destination = Location()
previous_predator_destination = Location(0,0)
destination_circle = None
ambush_circle = None
current_experiment_name = ""
prey_entered_step = Step()


# TIMER AND KILL SWITCH INIT
controller_timer = Timer(5.0)
# surge_timer = Timer(0.5)        # TODO: check this may Not work
controller_kill_switch = 0
SURGE_ROTATION = -2000

# EXPERIMENT FUNCTIONS
def get_experiment_folder(experiment_name):
    return experiment_log_folder + "/" + experiment_name.split('_')[0] + "/" + experiment_name


def get_episode_folder(experiment_name, episode_number):
    return get_experiment_folder(experiment_name) + f"/episode_{episode_number:03}"


def get_episode_file(experiment_name, episode_number):
    return get_episode_folder(experiment_name, episode_number) + f"/{experiment_name}_episode_{episode_number:03}.json"


def get_experiment_file(experiment_name, current_experiment_name):
    return get_experiment_folder(experiment_name) + current_experiment_name + "_experiment.json"


def on_capture(frame:int):
    print("PREY CAPTURED")
    # controller.set_behavior(0)


def on_prey_entered_arena():
    global prey_entered_step
    print("PREY ENTERED ARENA")
    global episode_in_progress, controller_timer
    episode_in_progress = True
    controller_timer.reset()
    prey_entered_step = prey.step
    # controller_timer = Timer(5.0)


def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(1.0)
        predator.step = step
    else:
        prey.is_valid = Timer(1.0) # TODO: may need to tune this
        prey.step = step

        # Ambush predator only
        if episode_in_progress:
            AmbushManager.prey_state = prey.step.location.dist(world.cells[AmbushManager.current_ambush_cell].location) <= ambush_region
        else:
            AmbushManager.prey_state = 0 # TODO: check this no surge when ep in progress
        # surge_timer.reset()


def on_experiment_started(experiment):
    global current_predator_destination, destination_circle
    print("Experiment started:", experiment)
    experiments[experiment.experiment_name] = experiment.copy()

    controller.pause()
    # current_predator_destination = predator.step.location
    # destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)


def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name
    current_experiment_name = parameters.experiment_name
    # episode_in_progress = True


def on_episode_finished(m):
    global episode_in_progress, current_predator_destination, episode_count, df, current_predator_heading, prey_entered_step
    print("EPISODE FINISHED")
    controller.pause()              # TODO: do I need this
    episode_in_progress = False

    # write to pickle file - ambush_cell each ep, ambush cell select bias, dict start surge-end surge
    # print(get_experiment_folder(current_experiment_name))
    # pickle_file_path = f'/research/data/4ROBOT/RobotStrategyExtraData/{current_experiment_name}.pkl' # uploads each episode to make sure data gets recorded if experiment fails
    pickle_file_path = f"{get_experiment_folder(current_experiment_name)}/{current_experiment_name}.pkl"
    df = log_data(pickle_file_path, episode_count, "ambush_cell_id", AmbushManager.current_ambush_cell, df)  # TODO: check this
    df = log_data(pickle_file_path, episode_count, "bias", AmbushManager.bias, df)
    df = log_data(pickle_file_path, episode_count, "prey_entered_arena", prey_entered_step, df)  # TODO: check this
    prey_entered_step = Step()
    episode_count += 1

    # update ambush cell bias - loop through previous trajectory
    experiment_state = experiment_service.get_experiment(current_experiment_name)
    last_episode_file = get_episode_file(current_experiment_name, experiment_state.episode_count-1)
    last_trajectory = Episode.load_from_file(last_episode_file).trajectories.get_agent_trajectory("mouse_0")
    last_trajectory_np = last_trajectory.get('location').to_numpy_array() # TODO: May need to update cellworld
    if last_trajectory_np.shape[0] != 0:
        AmbushManager.update_bias(last_trajectory_np)                           # plots new bias

    # select ambush cell for next episode
    AmbushManager.current_ambush_cell = AmbushManager.select_random_cell(list(surge_cell_dict.keys()))
    print(f"Ambush cell selected: {AmbushManager.current_ambush_cell}")
    print(f"Updated bias: {AmbushManager.bias}")

    # send robot to new ambush cell
    controller.resume()
    current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location
    current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
    AmbushManager.state = "AMBUSH"
    controller.set_destination(current_predator_destination, current_predator_heading)     # set destination
    controller_timer.reset()                                                                # reset controller timer
    destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)
    ambush_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color='red')


# class AgentData:
#     def __init__(self, agent_name: str):
#         self.is_valid = None  # timers for predator and prey updates
#         self.step = Step()
#         self.step.agent_name = agent_name

# PLOT FUNCTIONS
def update_agent_positions():
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


def on_keypress(event):
    """
    Sets up keyboard intervention based on key presses.
    """
    global running, current_predator_destination, controller_timer, controller_kill_switch, current_predator_heading
    key_actions = {
        "p": ("pause", controller.pause, 0),
        "r": ("resume", controller.resume, 1),
        "m": ("move", controller.resume, 1, True)
    }
    action = key_actions.get(event.key)
    if action:
        print(action[0])                    # print string associated with action
        action[1]()                         # change controller instance state
        controller_kill_switch = action[2]    # change controller_kill_switch variable assignment

        if len(action) > 3 and action[3] and not episode_in_progress:
            controller.pause()
            AmbushManager.select_random_cell(list(surge_cell_dict.keys()))
            print(f"Ambush cell selected: {AmbushManager.current_ambush_cell}")
            current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location
            current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
            AmbushManager.state = "AMBUSH"
            controller.set_destination(current_predator_destination, current_predator_heading)
            destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)
            controller_timer.reset()
            controller.resume()

            ambush_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color='red')


def on_click(event):
    """
    Right-click to start experiment
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
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)


# AMBUSH MANAGER
class AmbushManager:
    current_ambush_cell = None   # cell_id
    bias = {32: 1e-9, 277: 1e-9, 253: 1e-9, 269: 1e-9, 173: 1e-9}
    decay_rate = 0.75        # todo: check mod decay rate
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
    def update_bias(cls, prey_trajectory):
        print("BIAS UPDATE")
        # Decay the scores for each ambush cell
        for ambush_cell_id in cls.bias.keys():
            cls.bias[ambush_cell_id] *= cls.decay_rate
        print("decay", cls.bias)
        print(prey_trajectory)
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
            print("32 max")
            cls.bias[32] = second_max_value

        # saturation limit
        for ambush_cell_id in cls.bias.keys():
            if cls.bias[ambush_cell_id] > 10:
                cls.bias[ambush_cell_id] = 10

        # plot new bias for ambush cells
        cmap = plt.cm.Reds([weight / max(cls.bias.values()) for weight in cls.bias.values()])
        for i, ambush_cell_id in enumerate(cls.bias.keys()):
            display.cell(cell=world.cells[ambush_cell_id], color=cmap[i])

        # plt.plot(prey_trajectory[:, 0], prey_trajectory[:,1])

    @classmethod
    def draw_ambush_zone(cls, radius=cell_size * 3):
        for ambush_cell_id in surge_cell_dict.keys():
            x = [world.cells[ambush_cell_id].location.x + radius * math.cos(angle * (math.pi / 180)) for angle in range(360)]
            y = [world.cells[ambush_cell_id].location.y + radius * math.sin(angle * (math.pi / 180)) for angle in range(360)]
            plt.plot(x,y, color = "grey", alpha = 0.5)


# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, explore_color)
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())

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
controller.set_behavior(0)

# KEYPRESS SETUP
cid1 = display.fig.canvas.mpl_connect('button_press_event', on_click)
cid_keypress = display.fig.canvas.mpl_connect('key_press_event', on_keypress)

# AMBUSH SETUP
ambush_circle = display.circle(current_predator_destination, 0.015, 'red')
# AmbushManager.current_ambush_cell = 32 # TODO: delete this just a test
AmbushManager.draw_ambush_zone()

print("PRESS M TO SET INITIAL AMBUSH CELL")
running = True
while running:

    # IF PAUSE DONT EXECUTE REST OF LOOP ROBOT STOPS MOVING
    if not controller_kill_switch or AmbushManager.current_ambush_cell == None:
        print("KILL SWITCH OR AMBUSH CELL NOT SET", controller_kill_switch, AmbushManager.current_ambush_cell)
        controller.set_destination(predator.step.location)
        controller.pause()
        update_agent_positions()
        previous_predator_destination = current_predator_destination
        continue

    ########## DETERMINE DESTINATION #####################
    # IF MOUSE IN AMBUSH REGION AND PREY.IS_VALID SET DESTINATION TO CORRECT SURGE CELL
    if AmbushManager.prey_state and prey.is_valid:
        # TODO: might have to add like an ambush reached stipulation depends on how I want surge to look (AmbushManager.state == "AMBUSH_REACHED")
        print("SURGE - PREY IN AMBUSH REGION")
        AmbushManager.state = "SURGE"
        if world.cells[AmbushManager.current_ambush_cell].location.y > prey.step.location.y:
            surge_location = world.cells[surge_cell_dict[AmbushManager.current_ambush_cell][1]].location
        else:
            surge_location = world.cells[surge_cell_dict[AmbushManager.current_ambush_cell][0]].location
        current_predator_destination = surge_location

    # ELSE SET DESTINATION BACK TO SELECTED AMBUSH CELL
    else:
        if AmbushManager.state == "SURGE":
            print("DRIVE BACK TO AMBUSH CELL")
        AmbushManager.state = "AMBUSH"
        current_predator_destination = world.cells[AmbushManager.current_ambush_cell].location

    ########## CHECK DESTINATION      ##########
    # CHECK IF CURRENT DESTINATION REACHED PAUSE ROBOT -> BUFFER
    # changed so that only buffer if surge cell otherwise
    if current_predator_destination.dist(predator.step.location) < (cell_size * 1) and AmbushManager.state == "SURGE":
        print("SURGE DESTINATION REACHED")
        # if AmbushManager.state == "AMBUSH":
        #     AmbushManager.state = "AMBUSH_REACHED"
        current_predator_destination = predator.step.location # TODO: added this without checking it
        controller.pause()

    ############## SEND DESTINATION ######################
    # IF NOT SEND ROBOT TO DESTINATION
    # todo: changed this logic without checking
    elif current_predator_destination != previous_predator_destination:
        print(f"CURRENT BEHAVIOR STATE: {AmbushManager.state}")
        # controller.pause()
        if AmbushManager.state == "AMBUSH":
            current_predator_heading = surge_cell_dict[AmbushManager.current_ambush_cell][2]
            controller.set_destination(current_predator_destination, surge_cell_dict[AmbushManager.current_ambush_cell][2])     # set destination

        else:
            current_predator_heading = SURGE_ROTATION
            controller.set_destination(current_predator_destination, SURGE_ROTATION) # TODO: check that this is sent correctly

        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=explore_color)
        controller_timer.reset()
        controller.resume()

    elif not controller_timer:
        controller.set_destination(current_predator_destination, current_predator_heading)  # resend destination
        # print("reset timer")
        controller_timer.reset()

    # PLOT AGENT STATES
    update_agent_positions()
    previous_predator_destination = current_predator_destination # TODO: hfdjksfhdjks
    sleep(0.1)

controller.unsubscribe()
controller.stop()

