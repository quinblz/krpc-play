import math
from numpy.linalg import norm
from common.node import execute_next_node

class Maneuver():
    def __init__(self, conn, **kwargs):
        self.conn = conn
        ksc = conn.space_center
        self.vessel = ksc.active_vessel
        self.target = ksc.target_vessel or ksc.target_body

    def TWR(self):
        return self.available_thrust() / self.mass()

    def burn_time(self, delta_v, against_g=0):
        m0 = self.mass()
        F = self.available_thrust()

        if against_g:
            return norm(delta_v) / (F / m0 - against_g)

        Isp = self.vessel.specific_impulse * 9.82
        if not Isp:
            return float('inf')
        m1 = m0 / math.exp(norm(delta_v)/Isp)
        flow_rate = F / Isp
        delta_t = (m0 - m1) / flow_rate
        return delta_t

    def connect_streams(self):
        pass

    def execute_next_node(self):
        execute_next_node(self.conn)

    def execute_next_nodect_streams(self):
        pass
