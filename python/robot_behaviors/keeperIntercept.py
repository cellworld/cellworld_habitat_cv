import numpy as np
import pandas as pd
import pickle



file_name = '/research/cellworld_habitat_cv/python/robot_behaviors/botEvade_highways.pkl'
with open(file_name, 'rb') as f:
    loaded_array = pickle.load(f)
botEvade_north = loaded_array[0]
botEvade_south = loaded_array[1]
print(botEvade_north)
print(botEvade_south)