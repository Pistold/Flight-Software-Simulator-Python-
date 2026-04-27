import numpy as np

def read_sensors(state, pos_noise=0.5, vel_noise=0.1):
    pos = state['position'] + np.random.normal(0, pos_noise)
    vel = state['velocity'] + np.random.normal(0, vel_noise)
    return {'position': pos, 'velocity': vel}

