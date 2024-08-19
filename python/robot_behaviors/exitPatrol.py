import pandas as pd
# TODO: test change made if in north cells only go between north and middle waypoint

from robot_util import *
""" 
Robot has two modes
1. chase: if prey.is_valid AND is in north or south cells
2. patrol: otherwise
"""
# TODO check on prey entered something is weird

# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
patrol, chase_visible_in_region, chase_out_region_or_hidden = "cyan", "red", "firebrick"
destination_circle_color = patrol

# UTIL SETUP


# TIMER AND KILL SWITCH INIT
controller_timer = Timer(5.0)
controller_kill_switch = 0

# EXPERIMENT/CONTROLLER SETUP
previous_predator_destination = Location()
current_predator_destination = Location()
episode_in_progress = False
experiment_log_folder = "/research/data"
current_experiment_name = ""
mode_data = []
df = pd.DataFrame()
prey_entered_step = Step()
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
    global episode_in_progress, controller_timer, prey_entered_step
    episode_in_progress = True
    controller_timer.reset()
    prey_entered_step = prey.step



def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(1.0)
        predator.step = step
    else:
        prey.is_valid = Timer(2.0)  # TODO: may need to tune this
        prey.step = step



def on_experiment_started(experiment):
    global current_predator_destination, destination_circle
    print("Experiment started:", experiment)
    experiments[experiment.experiment_name] = experiment.copy()



def on_episode_started(parameters):
    global episode_in_progress, current_experiment_name
    current_experiment_name = parameters.experiment_name


def on_episode_finished(m):
    global episode_in_progress, current_predator_destination, episode_count, mode_data, df, prey_entered_step # TODO: FIX HOW EP COUNT IS AQUIRED
    print("EPISODE FINISHED")
    controller.pause()
    episode_in_progress = False

    # write to pickle file - (mode, start_frame) # TODO: test extra data record
    pickle_file_path = f"{get_experiment_folder(current_experiment_name)}/{current_experiment_name}.pkl"
    df = log_data(pickle_file_path, episode_count, "prey_entered_arena", prey_entered_step, df)  # TODO: check this
    prey_entered_step = Step()
    episode_count += 1

    # set destination to patrol path waypoint
    ep_manager.patrol_waypoint = random.choice(list(patrol_path.values()))       # select random waypoint in patrol path
    print(f"NEW waypoint cell selected: {ep_manager.patrol_waypoint}")
    current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
    destination_circle_color = patrol
    ep_manager.mode = 'not active'    # this is chase because what to call generate pattern function during control loop
    ep_manager.previous_side = next((k for k, v in patrol_path.items() if v == ep_manager.patrol_waypoint), None)
    controller.set_destination(current_predator_destination)
    destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=destination_circle_color)
    controller_timer.reset()
    controller.resume()
    # todo: make more functions for reptitive code like set destination

# PLOT FUNCTIONS
def update_agent_positions():
    prey_colors = {"north": "cyan", "south": "lime", "none": "magenta"}
    if prey.is_valid:
        display.agent(step=prey.step, color=prey_colors[ep_manager.current_side], size=10)
    else:
        display.agent(step=prey.step, color="gray", size=10)
    if predator.is_valid:
        display.agent(step=predator.step, color="red", size=10)
    else:
        display.agent(step=predator.step, color="gray", size=10)
    display.fig.canvas.draw_idle()
    display.fig.canvas.start_event_loop(0.001)
    sleep(0.1)


def draw_strategy_features():
    # patrol path waypoints
    for id in patrol_path.values():
        xn = world.cells[id].location.x
        yn = world.cells[id].location.y
        plt.scatter(xn, yn, color ='purple',  edgecolor = 'black',  s = 150, marker = '*', alpha = 0.75, zorder = 1)
    # N/S cells and chase regions
    for id in ep_manager.north_side:
        x = world.cells[id].location.x
        y = world.cells[id].location.y
        if id in ep_manager.north_chase_region:
            plt.scatter(x, y, color='blue', alpha=0.5, s=300, marker='h', zorder = 0)
        else:
            plt.scatter(x, y, color='blue', alpha=0.25, s=300, marker='h', zorder = 0)
    for id in ep_manager.south_side:
        x = world.cells[id].location.x
        y = world.cells[id].location.y
        if id in ep_manager.south_chase_region:
            plt.scatter(x, y, color ='green', alpha=0.5, s=300, marker='h', zorder = 0)
        else:
            plt.scatter(x, y, color='green', alpha=0.25, s=300, marker='h', zorder = 0)


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
            ep_manager.patrol_waypoint = random.choice(list(patrol_path.values()))       # select random waypoint in patrol path
            print(f"NEW waypoint cell selected: {ep_manager.patrol_waypoint}")
            current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
            destination_circle_color = patrol
            ep_manager.mode = 'not active'
            ep_manager.previous_side = next((k for k, v in patrol_path.items() if v == ep_manager.patrol_waypoint), None)
            controller.set_destination(current_predator_destination)
            destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=destination_circle_color)
            controller_timer.reset()


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



# EXITPATROL MANAGER
class ExitPatrolManager:
    def __init__(self, world):
        self.world = world
        self.cell_size = world.implementation.cell_transformation.size
        self.north_chase_region, self.south_chase_region = self.get_chase_regions()
        self.north_side, self.south_side = self._get_side()
        self.chase_region_dict = {'north': self.north_chase_region, 'south': self.south_chase_region, 'none': []}
        self.side_dict = {'north': self.north_side, 'south': self.south_side, 'none': []}
        self.patrol_waypoint = None # north, middle, or south
        self.mode = 'chase' # side patrol, direct chase, patrol, patrol reached

        # prey state machine
        self.prey_id = -1
        self.current_region = 'none'    # 'north', 'south', 'none'
        self.current_side = 'none'      # 'north', 'south', 'none'
        self.previous_side = 'none'

    def update_state(self, prey_id):
        self.prey_id = prey_id
        new_side = self._determine_side()
        if new_side != "none":
            self.previous_side = new_side
        self.current_side = new_side
        self.current_region = self._determine_region()
        # print("UPDATE_STATE", prey_id in self.north_side, self.prey_id)

    def get_chase_regions(self):
        north_chase_region = []
        south_chase_region = []
        last_cell_location = self.world.cells[330].location
        ub = last_cell_location.y + self.cell_size
        lb = last_cell_location.y - self.cell_size
        xb = self.world.cells[227].location.x

        for cell in self.world.cells.free_cells():
            distance = cell.location.dist(last_cell_location)
            if cell.location.x > xb and distance <= self.cell_size * 8:
                if cell.location.y <= lb:
                    south_chase_region.append(cell.id)
                elif cell.location.y >= ub:
                    north_chase_region.append(cell.id)

        return north_chase_region, south_chase_region

    def _get_side(self):
        north_cell_ids = []
        south_cell_ids = []
        start_location = self.world.cells[0].location
        for cell in self.world.cells.free_cells():
            if cell.location.dist(start_location) > self.cell_size * 3:
                if cell.location.y > 0.5:
                    north_cell_ids.append(cell.id)
                elif cell.location.y < 0.5:
                    south_cell_ids.append(cell.id)
        return north_cell_ids, south_cell_ids

    def _determine_region(self):
        if self.prey_id in self.north_chase_region:
            return 'north'
        elif self.prey_id in self.south_chase_region:
            return 'south'
        return 'none'

    def _determine_side(self):
        if self.prey_id in self.north_side:
            return 'north'
        elif self.prey_id in self.south_side:
            return 'south'
        return 'none'


# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, patrol, zorder = 300)
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

# EXIT PATROL SETUP
patrol_path = {'north': 274, 'middle': 326, 'south': 268}
ep_manager = ExitPatrolManager(world)
previous_mode = ep_manager.mode
draw_strategy_features()

# KEYPRESS SETUP
cid1 = display.fig.canvas.mpl_connect('button_press_event', on_click)
cid_keypress = display.fig.canvas.mpl_connect('key_press_event', on_keypress)

print("PRESS M TO SET PATROL WAYPOINT")
running = True

while running:
    # print(world.cells.find(prey.step.location))
    if episode_in_progress:
        ep_manager.update_state(world.cells.find(prey.step.location))


    # IF PAUSE DONT EXECUTE REST OF LOOP ROBOT STOPS MOVING
    if not controller_kill_switch:
        # print("KILL SWITCH OR PATROL CELL NOT SET")
        controller.set_destination(predator.step.location)
        controller.pause()
        update_agent_positions()
        previous_predator_destination = current_predator_destination
        continue
    ########## DETERMINE DESTINATION ##########
    # CHASE
    # print(prey.is_valid, ep_manager.current_side, episode_in_progress)
    if prey.is_valid and ep_manager.current_side != "none" and episode_in_progress:
        # direct pursuit
        if ep_manager.current_region != "none":
            ep_manager.mode = "direct_chase"
            current_predator_destination = prey.step.location
            destination_circle_color = chase_visible_in_region

        # random movement in region
        # TODO: just changed this to strategic patrol instead
        else:
            # select side patrol cell
            if ep_manager.mode != "side_patrol": # or ep_manager.mode != "patrol":
                ep_manager.mode = "side_patrol"
                ep_manager.patrol_waypoint = get_patrol_side_waypoint(ep_manager.patrol_waypoint, ep_manager.current_side, patrol_path)
                current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
                destination_circle_color = chase_out_region_or_hidden

    # PATROL
    elif episode_in_progress:
        # next patrol cell in pattern
        if ep_manager.mode == "patrol_reached":
            print("PATROL REACHED")
            ep_manager.mode = "patrol"
            pattern_key = next(pattern_iterator)
            ep_manager.patrol_waypoint = patrol_path[pattern_key]
            print(f"Continue Next Waypoint: {pattern_key}")
            current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
        # generate pattern from previous chase region
        elif ep_manager.mode != "patrol":
            print("PATROL MODE")
            ep_manager.mode = "patrol"
            pattern_iterator = generate_pattern(ep_manager.previous_side)
            pattern_key = next(pattern_iterator)
            ep_manager.patrol_waypoint = patrol_path[pattern_key]
            print(f"Patrol Pattern Start: {ep_manager.previous_side}")
            print(f"Next Waypoint: {ep_manager.patrol_waypoint}")
            current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
        destination_circle_color = patrol


    ########## SEND DESTINATION      ##########
    if current_predator_destination != previous_predator_destination:
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= destination_circle_color)
        controller.set_destination(current_predator_destination)
        controller_timer.reset()
        controller.resume()

    ########## CHECK DESTINATION      ##########
    if current_predator_destination.dist(predator.step.location) < (cell_size * 1):
        # print("Destination reached")
        if ep_manager.mode == "side_patrol" and episode_in_progress: # todo: this was the main issue
            ep_manager.mode = "side_patrol_reached"
        elif ep_manager.mode == "patrol" and episode_in_progress:
            ep_manager.mode = "patrol_reached"

        controller.pause()
        current_predator_destination = predator.step.location
        controller.set_destination(current_predator_destination)
        controller_timer.reset()
        if episode_in_progress: # todo: this is key to stopping robot pause it when want stop
            controller.resume()

    elif not controller_timer:
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= destination_circle_color)
        controller.set_destination(current_predator_destination)  # resend destination
        controller_timer.reset()

    # record mode
    # if previous_mode != ep_manager.mode:
    #     mode_data.append((prey.step.frame, ep_manager.mode))
    #     previous_mode = ep_manager.mode

    # PLOT AGENT STATES
    update_agent_positions()
    previous_predator_destination = current_predator_destination
    sleep(0.1)

controller.unsubscribe()
controller.stop()