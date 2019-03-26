import math
from numpy.linalg import norm
from common.node import execute_next_node

class Maneuver():
    def __init__(self, conn, **kwargs):
        self.conn = conn
        ksc = conn.space_center
        self.vessel = ksc.active_vessel
        self.target = ksc.target_vessel or ksc.target_body

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

    def execute_next_node(self):
        execute_next_node(self.conn)

    def TWR(self):
        return self.available_thrust() / self.mass()

    #### Connected functions ###

    def apoapsis(self):
        self.apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis')
        return self.apoapsis()

    def apoapsis_altitude(self):
        self.apoapsis_altitude = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        return self.apoapsis_altitude()

    def available_thrust(self):
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')
        return self.available_thrust()

    def eccentricity(self):
        self.eccentricity = self.conn.add_stream(getattr, self.vessel.orbit, 'eccentricity')
        return self.eccentricity()

    def latitude(self):
        self.latitude = self.conn.add_stream(getattr, self.vessel.flight(), 'latitude')
        return self.latitude()

    def longitude(self):
        self.longitude = self.conn.add_stream(getattr, self.vessel.flight(), 'longitude')
        return self.longitude()
    
    def mass(self):
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        return self.mass()

    def mean_altitude(self):
        self.mean_altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
        return self.mean_altitude()

    def semi_major_axis(self):
        self.semi_major_axis = self.conn.add_stream(getattr, self.vessel.orbit, 'semi_major_axis')
        return self.semi_major_axis()

    def situation(self):
        self.situation = self.conn.add_stream(getattr, self.vessel, 'situation')
        return self.situation()

    def surface_altitude(self):
        self.surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')
        return self.surface_altitude()

    def ut(self):
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        return self.ut()

    def abort(self):
        self.abort = self.conn.add_stream(getattr, self.vessel.control, 'abort')
        return self.abort()

    def brakes(self):
        self.brakes = self.conn.add_stream(getattr, self.vessel.control, 'brakes')
        return self.brakes()