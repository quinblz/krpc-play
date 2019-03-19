from common.node import execute_next_node

class Maneuver():
    def __init__(self, conn, **kwargs):
        self.conn = conn
        ksc = conn.space_center
        self.vessel = ksc.active_vessel
        self.target = ksc.target_vessel or ksc.target_body

    def burn_time(self, delta_v):
        F = self.vessel.available_thrust
        Isp = self.vessel.specific_impulse * 9.82
        m0 = self.mass
        #TODO: handle Isp = 0
        m1 = m0 / math.exp(delta_v/Isp)
        flow_rate = F / Isp
        delta_t = (m0 - m1) / flow_rate
        return delta_t

    def execute_next_node(self):
        execute_next_node(self.conn)
