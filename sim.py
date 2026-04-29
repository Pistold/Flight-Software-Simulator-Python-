GRAVITY = -9.81
MASS = 10.0  # kg

def update_physics(state, thrust, dt):
    net_force = thrust + MASS * GRAVITY
    acc = net_force / MASS
    state['velocity'] += acc * dt
    state['position'] += state['velocity'] * dt
    return state