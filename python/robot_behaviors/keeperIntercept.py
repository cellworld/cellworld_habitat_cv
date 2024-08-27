import cellworld

from robot_util import *
# TODO: record vector, agent distance to intercept, decision


# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)
explore_color = "cyan"

# ROBOT WORLD SETUP (for robot paths)
robot_world = World.get_from_parameters_names("hexagonal", "canonical")
occlusion = Cell_group_builder.get_from_name("hexagonal", occlusions + ".occlusions.robot")
robot_world.set_occlusions(occlusion)
pb = Paths_builder.get_from_name(world_configuration_name="hexagonal", occlusions_name=occlusions, path_type_name="astar.robot")
p = Paths(pb,  robot_world)
robot_world_free_cells = robot_world.cells.free_cells().get("location").to_numpy_array()
robot_world_cells = robot_world.cells.get("location").to_numpy_array()


# robot_path = p.get_path(robot_world.cells[33], robot_world.cells[330]).get('location').to_numpy_array()

# CLUSTER SETUP
file_name = '/research/cellworld_habitat_cv/python/robot_behaviors/botEvade_highways.pkl'
with open(file_name, 'rb') as f:
    loaded_array = pickle.load(f)
botEvade_north = loaded_array[0]
botEvade_south = loaded_array[1]

# print(botEvade_north)
# print(botEvade_south)


# INTERCEPT CALCULATION FUNCTIONS
def distance_to_intercept_point(route: np.array, start_index: int = 0, end_index: int = -1):
    """Input a route and find distance between two specified points on path"""
    if end_index < 0:
        end_index = len(route) - 1  # Adjust to include the endpoint when default is used

    if start_index <= end_index:
        distances = np.sqrt(np.sum(np.diff(route, axis=0)**2, axis=1))
        total_distance = np.sum(distances[start_index:end_index])
        return total_distance
    else:
        print("distance_to_intercept: function start index was greater than end_index on route")
        return 0


def closest_open_cell(current_id: int, world_cells = robot_world_cells, world_free_cells = robot_world_free_cells):
    """Use if tracker says agent is in an occluded cell"""
    if robot_world.cells[current_id].occluded:
        print("closest_open_cell: start or end location occluded retuning closest cell id")
        current_location = np.array([robot_world.cells[current_id].location.x, robot_world.cells[current_id].location.y])
        distances = np.linalg.norm(world_free_cells - current_location, axis=1)
        closest_index_in_free = np.argmin(distances)
        closest_free_cell = world_free_cells[closest_index_in_free]
        closest_index_in_cells = np.where((world_cells == closest_free_cell).all(axis=1))[0][0]
        return closest_index_in_cells
    else:
        return current_id


def get_robot_path(start_cell_location: cellworld.Location, end_cell_id: int):
    start_cell_id = closest_open_cell(robot_world.cells.find(start_cell_location))
    end_cell_id = closest_open_cell(end_cell_id)
    return p.get_path(robot_world.cells[start_cell_id], robot_world.cells[end_cell_id]).get('location').to_numpy_array()


def mouse_is_close_to_highway(current_location: np.array, route: np.array, threshold_distance = cell_size * 3):
    x, y = current_location[0], current_location[1]
    distances = np.sqrt((route[:, 0] - x) ** 2 + (route[:, 1] - y) ** 2)
    min_index = np.argmin(distances)
    closest_point = route[min_index]
    if distances[min_index] < threshold_distance:
        is_close = True
    else:
        is_close = False

    print("mouse_is_close_to_highway: Closest point on the trajectory:", closest_point)
    print("mouse_is_close_to_highway: Distance to closest point:", distances[min_index])
    return is_close, min_index






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


def draw_clusters(cluster_paths):
    colors = ['mediumseagreen', 'gold']
    for i, path in enumerate(cluster_paths):
        # plt.plot(path[:, 0], path[:, 1], color = colors[i] , alpha = 0.5, linestyle = "-", linewidth = 10, zorder = 1, marker = '.')
        plt.plot(path[:, 0], path[:, 1], color = colors[i] , alpha = 0.5, zorder = 1, marker = '.')

# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, explore_color)
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())
draw_clusters([botEvade_north, botEvade_south])

# TESTING
# plot robot path
print('test robot path')
robot_test_location = Location(0.11991367265558575,0.6947008349929531)
robot_intercept_route = get_robot_path(world.cells[0].location, 330)  # robot path -tested
robot_distance_to_intercept_point = distance_to_intercept_point(robot_intercept_route) # robot distance to intercept point -tested
print(f"robot distance: {robot_distance_to_intercept_point}")

# mouse path
mouse_test_path = np.array( [[0.446077  ,0.0763752],
                             [0.447828 , 0.0761795],
                             [0.446077 , 0.0763752],
                             [0.453097 , 0.0756173],
                             [0.44935  , 0.0760883],
                             [0.453097 , 0.0756173],
                             [0.454875  ,0.0754462],
                             [0.457821  ,0.0754466],
                             [0.459495  ,0.0755834],
                             [0.467372  ,0.0756944],
                             [0.465333  ,0.0755239],
                             [0.467372  ,0.0756944],
                             [0.47568   ,0.0758724]])

# determine closest highway to mouse based on observation
close_to_highway1, mouse_closest_index_highway1 = mouse_is_close_to_highway(mouse_test_path[0, :], botEvade_north)
close_to_highway2, mouse_closest_index_highway2 = mouse_is_close_to_highway(mouse_test_path[0, :], botEvade_south)
plt.scatter(botEvade_north[mouse_closest_index_highway1][0], botEvade_north[mouse_closest_index_highway1][1], color = 'mediumseagreen', edgecolors='k')
plt.scatter(botEvade_south[mouse_closest_index_highway2][0], botEvade_south[mouse_closest_index_highway2][1], color = 'gold', edgecolors='k')
# deteriment if heading towards highway

plt.plot(robot_intercept_route[:, 0], robot_intercept_route[:, 1], marker = ".")
# plt.plot(mouse_test_path[:, 0], mouse_test_path[:,1])
plt.scatter(mouse_test_path[0, 0], mouse_test_path[0,1], color ='k')

running = True
while running:


    # PLOT AGENT STATES
    update_agent_positions()
    sleep(0.1)