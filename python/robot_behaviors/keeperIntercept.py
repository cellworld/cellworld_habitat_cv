"""
To do list:
cluster analysis RT add middle lane (from Ambush)**
add search
add experiment logic
"""

from robot_util import *

# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
start_location = world.cells[0].location
display = Display(world, fig_size=(9.0 * .75, 8.0 * .75), animated=True)
destination_circle_color = "cyan"

# ROBOT WORLD SETUP
robot_world = World.get_from_parameters_names("hexagonal", "canonical")
occlusion = Cell_group_builder.get_from_name("hexagonal", occlusions + ".occlusions.robot")
robot_world.set_occlusions(occlusion)
pb = Paths_builder.get_from_name(world_configuration_name="hexagonal", occlusions_name=occlusions, path_type_name="astar.robot")
path_object = Paths(pb, robot_world)
robot_world_free_cells = robot_world.cells.free_cells().get("location").to_numpy_array()
robot_world_cells = robot_world.cells.get("location").to_numpy_array()

# CLUSTER SETUP
file_name = '/research/cellworld_habitat_cv/python/robot_behaviors/botEvade_highways.pkl'
with open(file_name, 'rb') as f:
    loaded_array = pickle.load(f)
botEvade_north, botEvade_south = loaded_array[0], loaded_array[1]
highway_list = [botEvade_north, botEvade_south]
highway_name_list = ['north', 'south']
highway_dict = {'north': botEvade_north, 'south': botEvade_south}                                                       # TYPE = LOCATION TRAJECTORY; value: numpy array of highway streamlines
cell_highway_dict = {'north': get_cell_route(botEvade_north, world), 'south': get_cell_route(botEvade_south, world)}    # TYPE = CELL_ID; value: list converting each streamline [x,y] to cell_id with repitition
cell_highway_set_dict = {key: np.array([[world.cells[cell_id].location.x, world.cells[cell_id].location.y] for cell_id in remove_duplicates_preserve_order(value)]) for key, value in cell_highway_dict.items()} # TYPE: LOCATION TRAJECTORY; value: list converting cell_highway_dict cell_id to location without repitition
coarse_highway_dict = {key: get_potential_intercept_point_highway_indices(cell_highway_dict[key], highway_dict[key], world) for key in highway_name_list} # TYPE = INDICES OF HIGHWAY; convert cell_highway to closest indices in highway
assert len(highway_dict['north']) > len(cell_highway_set_dict['north'])

search_cell_dict = {63: [89, 252], 89: [63, 236], 236: [89, 208], 208: [236, 63], 252: [63, 297], 297: [252, 236]}
search_cell_colors = plt.cm.get_cmap('tab10', len(search_cell_dict))
for i, (cell_id, search_cells) in enumerate(search_cell_dict.items()):
    plt.scatter(world.cells[cell_id].location.x, world.cells[cell_id].location.y, color=search_cell_colors(i), marker = '*', alpha=0.5)
    plt.text(world.cells[cell_id].location.x, world.cells[cell_id].location.y, f"{cell_id}", fontsize=12, ha='right', color = search_cell_colors(i))
# search_cell_bias_dict = {cell_id: 1e-9 for cell_id in search_cell_dict.keys()} # TODO: maybe add bias later

# CONSTANTS
PREY_SPEED = 0.75                       # Average speed of the prey
PREDATOR_SPEED = 0.23                   # 0.23 Average moving speed of the predator
PREY_OBSERVATION_BUFFER_LENGTH = 15

# TIMER AND KILL SWITCH INIT
controller_timer = Timer(5.0)
controller_kill_switch = 0

# EXPERIMENT/CONTROLLER SETUP
previous_predator_destination = Location()
current_predator_destination = Location()
episode_in_progress = False
experiment_log_folder = "/research/data"
current_experiment_name = ""
episode_count = 0


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


def on_prey_entered_arena():
    global episode_in_progress, controller_timer
    episode_in_progress = True
    controller_timer.reset()


def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(1.0)
        predator.step = step
    else:
        prey.is_valid = Timer(2.0)  # TODO: may need to tune this  (2.0 -> 3.0)
        prey.step = step


def on_experiment_started(experiment):
    global current_predator_destination, destination_circle
    print("Experiment started:", experiment)
    experiments[experiment.experiment_name] = experiment.copy()


def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name
    current_experiment_name = parameters.experiment_name


def on_episode_finished(m):
    global episode_in_progress, current_predator_destination, episode_count, df, current_predator_heading, prey_entered_step
    print("EPISODE FINISHED")
    controller.pause()
    episode_in_progress = False

    # select search cell for next episode
    search_cell_id = random.choice(list(search_cell_dict.keys()))
    reset_globals()

    # send robot to new ambush cell
    controller.resume()
    current_predator_destination = world.cells[search_cell_id].location
    controller.set_destination(current_predator_destination)     # set destination
    controller_timer.reset()                                                                # reset controller timer
    destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= 'magenta')


# KEEP INTERCEPT FUNCTIONS
def check_state(agent_position, agent_heading, size=cell_size):  # in file
    """ STEP 1. check if mouse is "on_highway" """
    for highway_name, highway in highway_dict.items():
        close_and_heading_bool, prey_closest_highway_point_index = mouse_is_near_and_heading_towards_highway(
            agent_position, highway, agent_heading, size * 3)

        if close_and_heading_bool:
            print(f"check_state: MOUSE IS ON {highway_name} HIGHWAY")
            prey_intercept_circle.set(center=(highway[prey_closest_highway_point_index][0],
                                              highway[prey_closest_highway_point_index][1]))
            plt.scatter(highway[prey_closest_highway_point_index][0],
                        highway[prey_closest_highway_point_index][1], color='blue', marker='h')

            return close_and_heading_bool, prey_closest_highway_point_index, highway_name, highway
    # close_close_and_heading bool always false
    return close_and_heading_bool, None, None, None


def get_intercept_info(close_and_heading_bool, prey_closest_highway_point_index, highway_name, highway):
    """STEP 2a. find robot distance to intercept 1 point (closest robot point) - iff mouse is "on" a highway
    RETURN:predator_mode, close_and_heading_bool, robot_destination, highway_name, highway index of robot_destination"""
    if close_and_heading_bool:
        robot_distance_to_highway_dict = {}         # index:distance
        robot_path_to_intercept_point_dict = {}     # index:path
        for i, highway_cell_id in enumerate(cell_highway_dict[highway_name]):
            robot_path_to_intercept_point_dict[i] = get_robot_interception_path(predator.step.location, highway_cell_id,
                                                                                robot_world, path_object,
                                                                                robot_world_cells,
                                                                                robot_world_free_cells)
            robot_distance_to_highway_dict[i] = distance_to_intercept_point(robot_path_to_intercept_point_dict[i])
        # find closest cell_id on highway to robot
        min_key, min_distance = min(robot_distance_to_highway_dict.items(), key=lambda x: x[1])
        end_intercept_point = coarse_highway_dict[highway_name][min_key]    # indice on highway (so streamline index)

        """STEP 2b. find mouse distance to intercept point"""
        prey_distance_to_intercept_point = distance_to_intercept_point(route=highway,
                                                                       start_index=prey_closest_highway_point_index,
                                                                       end_index=end_intercept_point)

        """STEP 2c. check if intercept 1 possible"""
        intercept_possible = calculate_intercept_time(prey_distance_to_intercept_point,
                                                      robot_distance_to_highway_dict[min_key],
                                                      PREY_SPEED, PREDATOR_SPEED)
        print(f"Is intercept 1 possible?: {intercept_possible}, prey distance: {prey_distance_to_intercept_point}, "
              f"predator distance: {min_distance}")

        if intercept_possible:
            predator_intercept_circle.set(center = (highway[end_intercept_point][0], highway[end_intercept_point][1]))
            return ("INTERCEPT", close_and_heading_bool,
                    Location(highway[end_intercept_point][0], highway[end_intercept_point][1]),
                    highway_name, end_intercept_point)
        else:
            """STEP 3. Check if intercept 2 possible"""
            for intercept_index, robot_distance in robot_distance_to_highway_dict.items():
                if intercept_index > prey_closest_highway_point_index:   # check all potential intercept points from where mouse starts
                    end_intercept_point = coarse_highway_dict[highway_name][intercept_index]
                    prey_distance_to_intercept_point = distance_to_intercept_point(route=highway,
                                                                                   start_index=prey_closest_highway_point_index,
                                                                                   end_index=end_intercept_point)
                    intercept_possible = calculate_intercept_time(prey_distance_to_intercept_point,
                                                                  robot_distance_to_highway_dict[intercept_index],
                                                                  PREY_SPEED, PREDATOR_SPEED)
                    if intercept_possible:
                        print(f"Is intercept 2 possible?: {intercept_possible}")
                        predator_intercept_circle.set(center=(highway[end_intercept_point][0],
                                                              highway[end_intercept_point][1]))
                        return ("INTERCEPT", close_and_heading_bool,
                                Location(highway[end_intercept_point][0], highway[end_intercept_point][1]),
                                highway_name, end_intercept_point)

            print(f"Is intercept 2 possible?: {intercept_possible}")
            return ("GOAL_INTERCEPT", close_and_heading_bool,
                    world.cells[326].location,
                    highway_name, cell_highway_set_dict[highway_name].shape[0])
    else:
        """4. mouse was not on highway"""
        return ("STEALTH_SEARCH", close_and_heading_bool,
                None,
                None, None)


def reset_globals():
    """Very bad code"""
    global predator_mode, previous_highway_name, close_and_heading_bool, predator_mode_int, prey_heading_vector
    global prey_smoothed_position, prey_observation_buffer_nparray
    predator_mode = "STEALTH_SEARCH"
    previous_highway_name = None
    close_and_heading_bool = None
    predator_mode_int = None        # destination index on coarse highway dict
    prey_heading_vector = None
    prey_smoothed_position = None
    prey_observation_buffer_nparray = np.empty((0, 2))


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
    global running, current_predator_destination, controller_timer, controller_kill_switch, destination_circle_color
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

        # can select new patrol waypoint when episode is not in progress
        if len(action) > 3 and action[3] and not episode_in_progress:
            search_cell_id = random.choice(list(search_cell_dict.keys()))
            reset_globals()
            controller.resume()
            current_predator_destination = world.cells[search_cell_id].location
            controller.set_destination(current_predator_destination)     # set destination
            controller_timer.reset()                                                                # reset controller timer
            destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= 'magenta')


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
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color='grey')



def draw_highways(highway_routes, colors=['mediumseagreen', 'gold']):
    """Draw highways"""
    for i, route in enumerate(highway_routes):
        plt.plot(route[:, 0], route[:, 1], color=colors[i], alpha=0.25, zorder=1, marker='.')


# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, destination_circle_color)
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())
draw_highways(highway_list)

# INTERCEPT SETUP
prey_intercept_circle = display.circle(Location(0,0), 0.01, 'green')
predator_intercept_circle = display.circle(Location(0,0), 0.01, 'blue')

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

# STATE MACHINE VARIABLES
predator_mode = "STEALTH_SEARCH"
previous_highway_name = None
close_and_heading_bool = None
predator_mode_int = None        # destination index on coarse highway dict
prey_heading_vector = None
prey_smoothed_position = None
prey_observation_buffer_nparray = np.empty((0, 2))


running = True
while running:
    if not controller_kill_switch:
        current_predator_destination = predator.step.location
        controller.set_destination(current_predator_destination)
        controller.pause()
        update_agent_positions()
        previous_predator_destination = current_predator_destination
        continue

    ############### PREY OBSERVATION #################
    if prey.is_valid and prey.step.location.dist(start_location) > cell_size * 3 and episode_in_progress:
        if prey_observation_buffer_nparray.shape[0] < PREY_OBSERVATION_BUFFER_LENGTH:
            prey_location_array = np.array([prey.step.location.x, prey.step.location.y])
            prey_observation_buffer_nparray = np.concatenate((prey_observation_buffer_nparray, prey_location_array.reshape(1, -1)))
        # OBSERVATION MADE
        if prey_observation_buffer_nparray.shape[0] == PREY_OBSERVATION_BUFFER_LENGTH:
            print("Prey Observation Made")

            # compute position and heading
            prey_heading_vector = estimate_heading(prey_observation_buffer_nparray)
            prey_smoothed_position = np.mean(prey_observation_buffer_nparray, axis=0)

            # check if on highway
            close_and_heading_bool, prey_closest_highway_point_index, highway_name, highway = check_state(
                prey_smoothed_position, prey_heading_vector)
            print(f'previous highway {previous_highway_name}; highway name {highway_name}')
            # TODO: slight flaw what if mouse passes robot; robot should not keep heading down the highway; PLAY WITH DISTANCE PASSING CRITERIA FOR ON HIGHWAY
            if (highway_name != previous_highway_name) or (not close_and_heading_bool):
                predator_mode, close_and_heading_bool, intercept_location, highway_name, end_intercept_point = (
                    get_intercept_info(close_and_heading_bool, prey_closest_highway_point_index, highway_name, highway))
                print(f"get_intercept_info, predator mode is: {predator_mode}, highway name: {highway_name}")

                if predator_mode == "STEALTH_SEARCH":
                    predator_mode_int = -1   # dont need for stealth search
                    current_closest_search_key = get_closest_search_key(predator.step.location, search_cell_dict, world)
                    destination_id = random.choice(search_cell_dict[current_closest_search_key])
                    current_predator_destination = world.cells[destination_id].location

                else:
                    current_predator_destination = intercept_location
                    predator_mode_int, _ = find_closest_point(np.array([intercept_location.x, intercept_location.y]),
                                                              cell_highway_set_dict[highway_name])

                previous_highway_name = highway_name

            prey_observation_buffer_nparray = np.empty((0, 2))

    ########## CHECK DESTINATION  REACHED ##########
    if current_predator_destination.dist(predator.step.location) < (cell_size * 1.5):
        if not episode_in_progress:
            controller.pause()
            current_predator_destination = predator.step.location

        elif predator_mode != "STEALTH_SEARCH":
            predator_mode_int, predator_mode = backtrack_trajectory(predator_mode_int, 5, predator_mode)
            if predator_mode != "STEALTH_SEARCH":
                x_predator, y_predator = cell_highway_set_dict[highway_name][predator_mode_int]
                current_predator_destination = Location(x_predator, y_predator)
                destination_circle_color = 'red'
            else:
                previous_highway_name = None
                print(f"highway traversed back to {predator_mode}")

        # need if here because chance function call above returns STEALTH_SEARCH
        if predator_mode == "STEALTH_SEARCH" and episode_in_progress:
            # find nearest key
            current_closest_search_key = get_closest_search_key(predator.step.location, search_cell_dict, world)
            # select new destination
            destination_id = random.choice(search_cell_dict[current_closest_search_key])
            current_predator_destination = world.cells[destination_id].location
            destination_circle_color = 'cyan'

    ########## SEND DESTINATION  ##########
    if current_predator_destination != previous_predator_destination and episode_in_progress:
        print(f"SET NEW DESTINATION, {predator_mode}")
        controller.pause()
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=destination_circle_color)
        controller.set_destination(current_predator_destination)
        controller_timer.reset()
        if episode_in_progress:
            # TODO: assuming episode will be in progress if the current_predator_destination != previous_predator_destination
            controller.resume()

    elif not controller_timer:
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= destination_circle_color)
        controller.set_destination(current_predator_destination)  # resend destination
        controller_timer.reset()

    # take care of pausing during stealth search
    if predator_mode == "STEALTH_SEARCH" and prey.is_valid:
        print(f"PREY SEEN IN STEALTH MODE, {predator_mode}")
        controller.pause()
    elif episode_in_progress:
        controller.resume()


    # PLOT AGENT STATES
    update_agent_positions()
    previous_predator_destination = current_predator_destination
    sleep(0.1)

controller.unsubscribe()
controller.stop()








