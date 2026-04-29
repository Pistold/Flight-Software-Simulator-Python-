import numpy as np
from collections import deque

class Sensor:
    def __init__(self, noise_std=0.5, bias=0.0, drift_rate=0.0,
                 resolution=None, delay_steps=0):
        self.noise_std = noise_std
        self.bias = bias
        self.drift_rate = drift_rate
        self.resolution = resolution
        self._drift = 0.0
        self._buffer = deque([0.0] * max(delay_steps, 1), maxlen=max(delay_steps, 1))

    def read(self, true_value, dt=0.05):
        # 1. Drift
        self._drift += np.random.normal(0, self.drift_rate) * dt

        # 2. Gaussian noise + bias + drift
        noisy = true_value + self.bias + self._drift + np.random.normal(0, self.noise_std)

        # 3. Quantization
        if self.resolution:
            noisy = round(noisy / self.resolution) * self.resolution

        # 4. Latency
        self._buffer.append(noisy)
        return self._buffer[0]


def read_sensors(state, dt=0.05):
    position_sensor = Sensor(noise_std=0.5, bias=0.2, drift_rate=0.005,
                              resolution=0.1, delay_steps=2)
    velocity_sensor = Sensor(noise_std=0.1, bias=0.0, drift_rate=0.002)

    return {
        'position': position_sensor.read(state['position'], dt),
        'velocity': velocity_sensor.read(state['velocity'], dt)
    }