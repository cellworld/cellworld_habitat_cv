import tkinter as tk
import sys
import os
from cellworld import *
from time import sleep
from tkinter import filedialog, simpledialog


if len(sys.argv) == 2:
    world_configration = sys.argv[1]
    occlusions_name = simpledialog.askstring(title="Map Editor", prompt="Worlds name:")
    if occlusions_name is None:
        exit()

elif len(sys.argv) == 3:
    world_configration = sys.argv[1]
    occlusions_name = sys.argv[2]


cellworld_data_folder = os.environ["CELLWORLD_CACHE"]
occlusions_file_name = "%s.%s.occlusions" % (world_configration, occlusions_name)
occlusions_path = os.path.join(cellworld_data_folder, "cell_group", occlusions_file_name)

def update_github_repository():
    commands = ["git pull", "git add *", 'git commit -m "world updated"', "git push"]
    cur_dir = os.getcwd()
    os.chdir(cellworld_data_folder)
    for command in commands:
        os.system(command)
    os.chdir(cur_dir)


# create_paths(world_configration, occlusions_name)
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

def on_click(button, cell):
    from matplotlib.backend_bases import MouseButton

    if button == MouseButton.LEFT:
        cell.occluded = not cell.occluded
        display.__draw_cells__()


def on_keypress(event):

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

    if event.key == "u":
        world.cells.occluded_cells().builder().save(occlusions_path)
        update_github_repository()
        # create_paths(world_configration, occlusions_name)
        # create_cell_visibility(world_configration, occlusions_name)
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
