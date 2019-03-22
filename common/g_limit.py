from common import clamp, surface_normal_gravity
from common.maneuver import Maneuver

class GLimited(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def g_limit(self, g):
        target_throttle = 0.0
        available_accl = self.available_thrust() / self.mass()
        target_accl = g * surface_normal_gravity
        if available_accl:
            target_throttle = target_accl / available_accl
            #TODO: notify if target_throttle > 0
            target_throttle = clamp(target_throttle)
        self.vessel.control.throttle = target_throttle