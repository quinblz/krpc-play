import math
from numpy.linalg import norm
from numpy import arccos, arcsin

from common import notify, clamp_radians
from common.maneuver import Maneuver

class ReturnToPlanet(Maneuver):
    '''
    Returns to atmospheric entry around the main body
    Precondition: in a nearly circular orbit around a moon
    '''
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def execute(self):
        vessel = self.vessel
        control = vessel.control
        orbit = vessel.orbit
        body = vessel.orbit.body
        rf = body.orbital_reference_frame
        flight = vessel.flight(rf)
        ksc = self.conn.space_center

        inclination_limit = 0.25 * math.pi # 45 degrees
        if abs(orbit.inclination) > inclination_limit:
            raise Exception('Inclination is too large')

        # TODO check eccentricity

        direction = flight.prograde
        direction = direction / norm(direction)
        angle_to_body_prograde = arccos(direction[1]) # prograde = (0,1,0)
        if flight.radial[1] > 0:
            angle_to_body_prograde = 2 * math.pi - angle_to_body_prograde

        #https://en.wikipedia.org/wiki/Hyperbolic_trajectory
        v_inf = body.orbit.speed
        v_inf_sq = v_inf * v_inf
        mu = body.gravitational_parameter
        r = orbit.semi_major_axis
        ecc = 1.0 + r * v_inf_sq / mu

        # departure angle relative to direction of periapsis
        theta = arccos(-1.0/ecc)

        true_anomaly = orbit.true_anomaly 
        true_anomaly += angle_to_body_prograde # traveling prograde
        true_anomaly += 0.5 * math.pi # position prograde relative to body
        true_anomaly += math.pi - theta
        true_anomaly = clamp_radians(true_anomaly)
        if orbit.true_anomaly < 0:
            true_anomaly -= 2 * math.pi
        ut = orbit.ut_at_true_anomaly(true_anomaly)

        v0 = math.sqrt(2.0 * mu / r + v_inf_sq)
        delta_v = v0 - orbit.orbital_speed_at(ut)

        node = control.add_node(ut, prograde=delta_v)

        next_body = body.orbit.body
        # all bodies in stock game with moons also have an atmosphere
        closest_approach = next_body.equatorial_radius + 0.7 * next_body.atmosphere_depth

        # TODO refine maneuver time by accounting for time to SOI change?
        notify('raising periapsis')
        while node.orbit.next_orbit.periapsis < closest_approach:
            node.prograde -= 1
        notify('refining maneuver time')
        prev_periapsis = node.orbit.next_orbit.periapsis * 2
        while prev_periapsis > node.orbit.next_orbit.periapsis:
            prev_periapsis = node.orbit.next_orbit.periapsis
            node.ut -= 1
        notify('refining periapsis')
        while node.orbit.next_orbit.periapsis < closest_approach:
            node.prograde -= 1
        while node.orbit.next_orbit.periapsis > closest_approach:
            node.prograde += 0.1
        
        self.execute_next_node()
        ksc.warp_to(ksc.ut + vessel.orbit.time_to_soi_change + 1)

if __name__ == "__main__":
    import krpc
    conn = krpc.connect()
    ReturnToPlanet(conn).execute()