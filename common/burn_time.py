import math
from common.maneuver import Maneuver

class BurnTime(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def burn_time(self, delta_v):
        F = self.vessel.available_thrust
        Isp = self.vessel.specific_impulse * 9.82
        m0 = self.mass()
        #TODO: handle Isp = 0
        m1 = m0 / math.exp(delta_v/Isp)
        flow_rate = F / Isp
        delta_t = (m0 - m1) / flow_rate
        return delta_t