from cellworld import *
from cellworld_controller_service import ControllerClient
from cellworld_experiment_service import ExperimentClient
from time import sleep
class AgentData:
    def __init__(self, agent_name: str):
        self.is_valid = None # timers for predator and prey updates
        self.step = Step()
        self.step.agent_name = agent_name


