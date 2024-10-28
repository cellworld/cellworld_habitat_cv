import tkinter as tk
import sys
import os
from cellworld import *
from time import sleep
from tkinter import filedialog, simpledialog


if len(sys.argv) == 2:
    world_configration = sys.argv[1]
    occlusions_name = simpledialog.askstring(title="Map Editor", prompt="World's name:")
    if occlusions_name is None:
        exit()

elif len(sys.argv) == 3:
    world_configration = sys.argv[1]
    occlusions_name = sys.argv[2]


cellworld_data_folder = os.environ["CELLWORLD_CACHE"]
cellworld_bin_folder = os.environ["CELLWORLD_BIN"]

occlusions_file_name = "%s.%s.occlusions" % (world_configration, occlusions_name)
occlusions_path = os.path.join(cellworld_data_folder, "cell_group", occlusions_file_name)


def run_commands(commands, folder):
    cur_dir = os.getcwd()
    os.chdir(folder)
    for command in commands:
        os.system(command)
    os.chdir(cur_dir)

def update_github_repository():
    commands = ["git pull", "git add *", 'git commit -m "world updated"', "git push"]
    run_commands(commands, cellworld_data_folder)


def create_paths(world_configration, occlusions_name, robot):
    paths_file_name = "%s.%s.astar" % (world_configration, occlusions_name)
    robot_str = ""
    if robot:
        paths_file_name += ".robot"
        robot_str = "-r"
    paths_path = os.path.join(cellworld_data_folder, "paths", paths_file_name)
    commands = ["./create_paths -c '%s' -o '%s' -of '%s' %s" % (world_configration, occlusions_name, paths_path, robot_str)]
    run_commands(commands, cellworld_bin_folder)


def create_robot_world(world_configration, occlusions_name):
    path_file_name = "%s.%s.occlusions.robot" % (world_configration, occlusions_name)
    paths_path = os.path.join(cellworld_data_folder, "cell_group", path_file_name)
    commands = ["./create_robot_occlusions -c '%s' -o '%s' -of '%s'" % (world_configration, occlusions_name, paths_path)]
    run_commands(commands, cellworld_bin_folder)


def create_visibility(world_configration, occlusions_name):
    visibility_file_name = "%s.%s.cell_visibility" % (world_configration, occlusions_name)
    paths_path = os.path.join(cellworld_data_folder, "graph", visibility_file_name)
    commands = ["./create_visibility -c '%s' -o '%s' -of '%s'" % (world_configration, occlusions_name, paths_path)]
    run_commands(commands, cellworld_bin_folder)


def create_predator_destinations(world_configration, occlusions_name):
    path_file_name = "%s.%s.predator_destinations" % (world_configration, occlusions_name)
    paths_path = os.path.join(cellworld_data_folder, "cell_group", path_file_name)
    commands = ["./create_predator_destinations -c '%s' -o '%s' -of '%s'" % (world_configration, occlusions_name, paths_path)]
    run_commands(commands, cellworld_bin_folder)


def create_spawn_locations(world_configration, occlusions_name):
    path_file_name = "%s.%s.spawn_locations" % (world_configration, occlusions_name)
    paths_path = os.path.join(cellworld_data_folder, "cell_group", path_file_name)
    commands = ["./create_spawn_locations -c '%s' -o '%s' -of '%s'" % (world_configration, occlusions_name, paths_path)]
    run_commands(commands, cellworld_bin_folder)

# create_cell_visibility(world_configration, occlusions_name)
# create_occlusions_robot(world_configration, occlusions_name)
# create_predator_destinations(world_configration, occlusions_name)
# create_robot_paths(world_configration, occlusions_name)
#
occlusions = Cell_group_builder()
if os.path.exists(occlusions_path):
    occlusions.load_from_file(occlusions_path)

world = World.get_from_parameters_names("hexagonal", "canonical")
world.set_occlusions(occlusions)
display = Display(world, animated=True)
display.fig.canvas.manager.set_window_title(occlusions_name)


def on_click(button, cell):
    from matplotlib.backend_bases import MouseButton

    if button == MouseButton.LEFT:
        cell.occluded = not cell.occluded
        display.__draw_cells__()


def on_keypress(event):
    global occlusions_name, occlusions_file_name, occlusions_path, display

    if event.key == "o":
        file_name = filedialog.askopenfile(filetypes=[("Cell Group file", "json")]).name
        o = Cell_group_builder().load_from_file(file_name)
        world.set_occlusions(o)
        display.__draw_cells__()

    if event.key == "s":
        file_name = filedialog.asksaveasfilename(filetypes=[("Cell Group file", "json"), ("Figure file", "pdf png")])
        if ".pdf" in file_name or ".png" in file_name:
            display.fig.savefig(file_name)
        else:
            world.cells.occluded_cells().builder().save(file_name)

    if event.key == "n":
        occlusions_name = simpledialog.askstring(title="Map Editor", prompt="New world's name:")
        occlusions_file_name = "%s.%s.occlusions" % (world_configration, occlusions_name)
        occlusions_path = os.path.join(cellworld_data_folder, "cell_group", occlusions_file_name)
        display.fig.canvas.manager.set_window_title(occlusions_name)

    if event.key == "u":
        print("Saving world changes...")
        world.cells.occluded_cells().builder().save(occlusions_path)
        print("occlusions saved to %s" % occlusions_path)
        print("Generating robot occlusions...")
        create_robot_world(world_configration, occlusions_name)
        print("Generating predator destinations...")
        create_predator_destinations(world_configration, occlusions_name)
        print("Generating paths...")
        create_paths(world_configration, occlusions_name, False)
        print("Generating robot paths...")
        create_paths(world_configration, occlusions_name, True)
        print("Generating cell visibility graph...")
        create_visibility(world_configration, occlusions_name)
        print("Generating spawn locations...") #needs visibility
        create_spawn_locations(world_configration, occlusions_name)
        print("Done")

    if event.key == "g":
        print("Updating repository...")
        update_github_repository()
        #
        # create_occlusions_robot(world_configration, occlusions_name)
        # create_predator_destinations(world_configration, occlusions_name)
        # create_robot_paths(world_configration, occlusions_name)


    if event.key == "c":
        for c in world.cells:
            c.occluded = False
        display._draw_cells__()

    if event.key == "q":
        exit(0)


map_text = display.ax.text(0, 1, "Label")


def on_mouse_move(location, cell=None):
    if cell:
        map_text.set_text(location.format("x:{x:.2f} y:{y:.2f} ") + cell.format("id:{id}"))
    else:
        map_text.set_text(location.format("x:{x:.2f} y:{y:.2f}"))


display.set_cell_clicked_event(on_click)
display.set_key_pressed_event(on_keypress)
display.set_mouse_move_event(on_mouse_move)
while True:
    sleep(.01)
    display.update()
