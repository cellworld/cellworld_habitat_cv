from robot_util import *

############ GLOBALS ASSIGNMENT ################
occlusions = "21_05"
world = World.get_from_parameters_names("hexagonal", "canonical", occlusions)
display = Display(world, fig_size=(9.0*.75, 8.0*.75), animated=True)

time_out = 1.0
predator = AgentData("predator")
prey = AgentData("prey")

####################################

def on_step(step: Step):
    if step.agent_name == "predator":
        predator.is_valid = Timer(time_out)
        predator.step = step
    else:
        prey.is_valid = Timer(time_out) # pursue when prey is seen
        prey.step = step


# CONNECT TO EXPERIMENT SERVER
experiment_service = ExperimentClient()
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
robot_location = world.cells[127].location

# controller.set_destination(robot_location)  # resend destination
running = True
while running:
    controller.set_destination(robot_location, 180.0)
    display.agent(step=predator.step, color="blue", size=10)
    display.update()
    sleep(0.1)
controller.unsubscribe()
controller.stop()