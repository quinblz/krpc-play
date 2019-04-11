import math

from common import wait, notify
from common.staging import StagingAware
from common.g_limit import GLimited
from common.maneuver import Maneuver

class Circularize(StagingAware, GLimited, Maneuver):
    def __init__(self, conn, target_heading=90.0, **kwargs):
        super().__init__(conn, **kwargs)
        self.target_heading = target_heading

    def circularization_delta_v(self, r):
        '''
        https://en.wikipedia.org/wiki/Vis-viva_equation
        '''
        orbit = self.vessel.orbit
        mu = orbit.body.gravitational_parameter
        a1 = orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        delta_v = v2 - v1
        return delta_v

    def execute(self):
        orbit = self.vessel.orbit
        if abs(orbit.apoapsis - orbit.periapsis) < 1e4 and orbit.periapsis_altitude > orbit.body.atmosphere_depth:
            return
        notify("Circularizing...")

        self.initial_apoapsis = self.apoapsis()
        control = self.vessel.control
        control.throttle = 0.0

        radius = max(self.apoapsis(), self.periapsis())
        delta_v = self.circularization_delta_v(radius)
        burn_time = self.burn_time(delta_v)
        ut = self.ut()
        apoapsis_ut = ut + min(orbit.time_to_apoapsis, orbit.time_to_periapsis)
        eccentricity = self.eccentricity()

        notify(f"delta_v: {delta_v}")
        notify(f"burn_time: {burn_time}")
        notify(f"initial_eccentricity: {eccentricity}")

        control.add_node(apoapsis_ut, prograde=delta_v)
        self.execute_next_node()
        notify(f"final_eccentricity: {self.eccentricity()}")

if __name__ == "__main__":
    import krpc
    conn = krpc.connect()
    Circularize(conn).execute()