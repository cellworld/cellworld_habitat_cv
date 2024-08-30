"""
To do list:
cluster analysis RT add middle lane (from Ambush)**

"""

from robot_util import *

# WORLD SETUP
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
cell_size = world.implementation.cell_transformation.size
display = Display(world, fig_size=(9.0 * .75, 8.0 * .75), animated=True)
explore_color = "cyan"

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
highway_list = [botEvade_north, botEvade_south]                 # highway streamlines; trajectory of locations
highway_name_list = ['north', 'south']
highway_dict = {'north': botEvade_north, 'south': botEvade_south}
cell_highway_dict = {'north': get_cell_route(botEvade_north, world), 'south': get_cell_route(botEvade_south, world)}
coarse_highway_dict = {key: get_potential_interecpt_point_highway_indices(cell_highway_dict[key], highway_dict[key], world) for key in highway_name_list}   # convert cell_highay to closest indices in highway
print(coarse_highway_dict)
# CONSTANTS
PREY_SPEED = 0.75  # Average speed of the prey
PREDATOR_SPEED = 0.23 #0.23  # Average speed of the predator




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


def draw_clusters(cluster_paths, colors = ['mediumseagreen', 'gold']):
    for i, path in enumerate(cluster_paths):
        plt.plot(path[:, 0], path[:, 1], color=colors[i], alpha=0.5, zorder=1, marker='.')


# AGENT SETUP
predator = AgentData("predator")
prey = AgentData("prey")
current_predator_destination = predator.step.location
destination_circle = display.circle(predator.step.location, 0.01, explore_color)
display.set_agent_marker("predator", Agent_markers.arrow())
display.set_agent_marker("prey", Agent_markers.arrow())
draw_clusters(highway_list)

# Test Plot Setup for Functions
mouse_test_path = np.array([
    [0.446077, 0.0763752],
    [0.447828, 0.0761795],
    [0.446077, 0.0763752],
    [0.453097, 0.0756173],
    [0.44935, 0.0760883],
    [0.453097, 0.0756173],
    [0.454875, 0.0754462],
    [0.457821, 0.0754466],
    [0.459495, 0.0755834],
    [0.467372, 0.0756944],
    [0.465333, 0.0755239],
    [0.467372, 0.0756944],
    [0.47568, 0.0758724]])
robot_test_location = world.cells[262].location
prey_position_window = mouse_test_path[0:10]
prey_heading_vector = estimate_heading(prey_position_window)
prey_smoothed_position = np.mean(prey_position_window, axis=0)

# plot predator position and prey position and aproximated heading
plt.arrow(prey_smoothed_position[0], prey_smoothed_position[1], prey_heading_vector[0] * 0.1, prey_heading_vector[1] * 0.1,
          head_width=0.02, head_length=0.02, fc='b', ec='blue')
plt.scatter(robot_test_location.x, robot_test_location.y, color = 'r')
plt.scatter(prey_smoothed_position[0], prey_smoothed_position[1], color ='b')
plt.plot(prey_position_window[:,0], prey_position_window[:,1], color = 'cyan', zorder = 1)


# 1. check if mouse "on_highway"
prey_heading_vector = estimate_heading(prey_position_window)
for highway_name, highway in highway_dict.items():
    close_and_heading_bool, prey_closest_highway_point_index = mouse_is_near_and_heading_towards_highway(
        prey_smoothed_position, highway, prey_heading_vector, cell_size * 3)
    if close_and_heading_bool:
        print(f"Mouse is on {highway_name} highway")
        plt.scatter(highway[prey_closest_highway_point_index][0], highway[prey_closest_highway_point_index][1], color = 'blue', marker = 'h')

# 2. find robot distance to intercept 1 point
robot_distance_to_highway_dict = {}         # index:distance
robot_path_to_intercept_point_dict = {}     # index:path
for i, highway_cell_id in enumerate(cell_highway_dict[highway_name]): # loop thorugh each cell id in highway

    robot_path_to_intercept_point_dict[i] = get_robot_interception_path(robot_test_location, highway_cell_id, robot_world, path_object, robot_world_cells, robot_world_free_cells)
    robot_distance_to_highway_dict[i] = distance_to_intercept_point(robot_path_to_intercept_point_dict[i]) # robot distance to intercept point -tested
print(robot_distance_to_highway_dict)
min_key, min_distance = min(robot_distance_to_highway_dict.items(), key=lambda x: x[1])
print(f"The minimum distance is {min_distance}, corresponding to highway cell index {min_key}.")
print(robot_distance_to_highway_dict[min_key])
plt.plot(robot_path_to_intercept_point_dict[min_key][:,0], robot_path_to_intercept_point_dict[min_key][:,1])
end_intercept_point = coarse_highway_dict[highway_name][min_key]
plt.scatter(highway[end_intercept_point][0], highway[end_intercept_point][1], color = 'purple', marker = '*')


# # 3. find mouse distance to intercept point
prey_distance_to_intercept_point = distance_to_intercept_point(route = highway, start_index = prey_closest_highway_point_index, end_index=end_intercept_point)
print(f"prey distance: {prey_distance_to_intercept_point}")
#
# # 4. check if intercept possible
intercept_possible = calculate_intercept_time(prey_distance_to_intercept_point, robot_distance_to_highway_dict[min_key], PREY_SPEED, PREDATOR_SPEED)
print(f"Is the intercept possible?: {intercept_possible}")

# if intercept not possible for intercept point 1 check the others
for intercept_index, robot_distance in robot_distance_to_highway_dict.items():
    if intercept_index > prey_closest_highway_point_index:   # check all potential intercept pint from where mouse starts
        end_intercept_point = coarse_highway_dict[highway_name][intercept_index]
        prey_distance_to_intercept_point = distance_to_intercept_point(route = highway, start_index = prey_closest_highway_point_index, end_index=end_intercept_point)
        intercept_possible = calculate_intercept_time(prey_distance_to_intercept_point, robot_distance_to_highway_dict[intercept_index], PREY_SPEED, PREDATOR_SPEED)
        if intercept_possible:
            plt.scatter(highway[end_intercept_point][0], highway[end_intercept_point][1], color = 'cyan', marker = '*')
            break
if intercept_possible:
    print("intercept_possible")
else:
    print('intercept not possible head to exit')



running = True
# prey_position_window = []
while running:
    # if prey.isvalid and (len(prey_position_window) == 0 or len(prey_position_window <= 15):
    #   prey_position_window.append([prey.step.location.x, prey.step.location.y])
    #else:
    # prey_position_window = []

    # PLOT AGENT STATES
    update_agent_positions()
    sleep(0.1)