import numpy as np
from sensors import read_sensors
from controller import PIDController
from sim import update_physics
import matplotlib.pyplot as plt

TARGET_ALTITUDE = 100.0  # meters
DT = 0.05                # timestep in seconds
DURATION = 20.0          # seconds

state = {'position': 0.0, 'velocity': 0.0}
pid = PIDController()

history = {'t': [], 'pos': [], 'thrust': []}

for step in range(int(DURATION / DT)):
    t = step * DT
    sensor_data = read_sensors(state)
    thrust = pid.compute(TARGET_ALTITUDE, sensor_data['position'], DT)
    thrust = max(0, min(thrust, 500))  # clamp actuator output
    state = update_physics(state, thrust, DT)

    history['t'].append(t)
    history['pos'].append(state['position'])
    history['thrust'].append(thrust)

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(history['t'], history['pos'], label='Altitude (m)')
ax1.axhline(TARGET_ALTITUDE, color='r', linestyle='--', label='Target')
ax1.set_ylabel('Altitude (m)'); ax1.legend()
ax2.plot(history['t'], history['thrust'], color='orange', label='Thrust (N)')
ax2.set_ylabel('Thrust (N)'); ax2.set_xlabel('Time (s)'); ax2.legend()
plt.tight_layout()
plt.show()