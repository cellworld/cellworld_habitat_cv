from robot_util import *
""" 
Robot has two modes
1. chase: if prey.is_valid AND is in north or south cells
2. patrol: otherwise
"""
# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
patrol, chase_visible_in_region, chase_out_region_or_hidden = "cyan", "red", "magenta"
destination_circle_color = patrol

# UTIL SETUP


# TIMER AND KILL SWITCH INIT
controller_timer = Timer(5.0)
controller_kill_switch = 0

# EXPERIMENT/CONTROLLER SETUP
previous_predator_destination = Location()
current_predator_destination = Location()
episode_in_progress = False

# EXPERIMENT FUNCTIONS
def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(1.0)
        predator.step = step
    else:
        prey.is_valid = Timer(2.0)  # TODO: may need to tune this
        prey.step = step
        ep_manager.update_state(prey.step)


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
        # plt.scatter(xn, yn, color ='purple',  edgecolor = 'black',  s = 150, marker = '*', alpha = 0.75)
    # N/S cells and chase regions
    for id in ep_manager.north_side:
        x = world.cells[id].location.x
        y = world.cells[id].location.y
        # if id in ep_manager.north_chase_region:
        #     plt.scatter(x, y, color='blue', alpha=0.5, s=300, marker='h')
        # else:
        #     plt.scatter(x, y, color='blue', alpha=0.25, s=300, marker='h')
    for id in ep_manager.south_side:
        x = world.cells[id].location.x
        y = world.cells[id].location.y
        # if id in ep_manager.south_chase_region:
        #     plt.scatter(x, y, color ='green', alpha=0.5, s=300, marker='h')
        # else:
        #     plt.scatter(x, y, color='green', alpha=0.25, s=300, marker='h')


def on_keypress(event):
    """
    Sets up keyboard intervention based on key presses.
    """
    global running, current_predator_destination, controller_timer, controller_kill_switch, destination_circle_color
    # key_actions = {
    #     "p": ("pause", controller.pause, 0),
    #     "r": ("resume", controller.resume, 1),
    #     "m": ("auto", controller.resume, 1, True)
    # }
    key_actions = {     # todo: delete once contoller setup
        "p": ("pause", 0),
        "r": ("resume", 1),
        "m": ("move", 1, 0, True)
    }
    action = key_actions.get(event.key)
    if action:
        print(action[0])                    # print string associated with action
        # action[1]()                         # change controller instance state
        # controller_kill_switch = action[2]    # change controller_kill_switch variable assignment

        # can select new patrol waypoint when episode is not in progress
        if len(action) > 3 and action[3] and not episode_in_progress:
            ep_manager.patrol_waypoint = random.choice(list(patrol_path.values()))       # select random waypoint in patrol path
            print(f"Waypoint cell selected: {ep_manager.patrol_waypoint}")
            current_predator_destination = world.cells[ep_manager.patrol_waypoint].location
            destination_circle_color = patrol
            # controller.set_destination(current_predator_destination)
            destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=destination_circle_color)
            # controller_timer.reset()


# EXITPATROL MANAGER
class ExitPatrolManager:
    def __init__(self, world):
        self.world = world
        self.cell_size = world.implementation.cell_transformation.size
        self.north_chase_region, self.south_chase_region = self.get_chase_regions()
        self.north_side, self.south_side = self._get_side()
        self.chase_region_dict = {'north': self.north_chase_region, 'south': self.south_chase_region, 'none': []}
        self.side_dict = {'north': self.north_side, 'south': self.south_side, 'none': []}
        self.patrol_waypoint = None # cell id
        self.mode = 'patrol' # patrol or chase

        # prey state machine -
        # self.prey_step = world.cells[0]
        self.current_region = 'none'    # 'north', 'south', 'none'
        self.current_side = 'none'      # 'north', 'south', 'none'
        self.previous_side = 'none'

    def update_state(self, prey_step):
        # self.prey_step = prey_step
        new_side = self._determine_side(prey_step)
        if new_side != "none":
            self.previous_side = new_side
        self.current_side = new_side
        self.current_region = self._determine_region(prey_step)

    def get_chase_regions(self):
        north_chase_region = []
        south_chase_region = []
        last_cell_location = self.world.cells[330].location
        ub = last_cell_location.y + self.cell_size
        lb = last_cell_location.y - self.cell_size
        xb = self.world.cells[227].location.x

        for cell in self.world.cells.free_cells():
            dist = cell.location.dist(last_cell_location)
            if cell.location.x > xb and dist <= self.cell_size * 8:
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

    def _determine_region(self, prey_step):
        if prey_step.id in self.north_chase_region:
            return 'north'
        elif prey_step.id in self.south_chase_region:
            return 'south'
        return 'none'

    def _determine_side(self, prey_step):
        if prey_step.id in self.north_side:
            return 'north'
        elif prey_step.id in self.south_side:
            return 'south'
        return 'none'


# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, patrol)
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())

# EXIT PATROL SETUP
patrol_path = {'north': 294, 'middle': 326, 'south': 289}
ep_manager = ExitPatrolManager(world)
draw_strategy_features()

# KEYPRESS SETUP
# cid1 = display.fig.canvas.mpl_connect('button_press_event', on_click)
cid_keypress = display.fig.canvas.mpl_connect('key_press_event', on_keypress)

current_prey_step = world.cells[250] # test
# prey.step = current_prey_step
ep_manager.update_state(current_prey_step)

print("PRESS M TO SET PATROL WAYPOINT")
running = True
i = 0
while running:

    # IF PAUSE DONT EXECUTE REST OF LOOP ROBOT STOPS MOVING
    if not controller_kill_switch:
        # print("KILL SWITCH OR AMBUSH CELL NOT SET")
        # controller.set_destination(predator.step.location)
        # controller.pause()

        update_agent_positions()
        previous_predator_destination = current_predator_destination

        # test
        if i == 0:
            print("KILL SWITCH OR AMBUSH CELL NOT SET")
            previous_predator_destination = world.cells[294].location
            a = select_random_cell(ep_manager.chase_region_dict[ep_manager.current_region], previous_predator_destination, 3, world)
            print(a)
            plt.scatter(a.x, a.y)
            i += 1
        continue

    ########## DETERMINE DESTINATION ##########
    # CHASE
    if prey.is_valid and ep_manager.current_side != "none":
        # direct pursuit
        if ep_manager.current_region != "none":
            current_predator_destination = prey.step.location
            destination_circle_color = chase_visible_in_region

        # random movement in region
        else:
            current_predator_destination = select_random_cell(ep_manager.chase_region_dict[ep_manager.current_side], previous_predator_destination, 3, world)
            destination_circle_color = chase_out_region_or_hidden
    # PATROL
    else:
        # follow pattern based previous chase region




    if current_predator_destination != previous_predator_destination:
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color=destination_circle_color )

    ########## SEND DESTINATION      ##########
    if current_predator_destination.dist(predator.step.location) < (cell_size * 1):
        print("Destination reached")
        # controller.pause()
        current_predator_destination = predator.step.location  # assign destination to current predator location (artificially reach goal when "close enough")

    elif not controller_timer:
        destination_circle.set(center=(current_predator_destination.x, current_predator_destination.y), color= destination_circle_color)
        # controller.set_destination(current_predator_destination)  # resend destination
        # controller_timer.reset()


    # PLOT AGENT STATES
    update_agent_positions()
    previous_predator_destination = current_predator_destination
    sleep(0.1)

# controller.unsubscribe()
# controller.stop()