from common import clamp, surface_normal_gravity

class GLimited():
    def __init__(self):
        self.vessel = None
        self.available_thrust = lambda: 0.0
        self.mass = lambda: 1.0

    def g_limit(self, g):
        target_throttle = 0.0
        available_accl = self.available_thrust() / self.mass()
        target_accl = g * surface_normal_gravity
        if available_accl:
            target_throttle = target_accl / available_accl
            #TODO: notify if target_throttle > 0
            target_throttle = clamp(target_throttle)
        self.vessel.control.throttle = target_throttle