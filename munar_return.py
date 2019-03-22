import math
from numpy.linalg import norm
from numpy import arccos, arcsin

from common import notify
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
        conn = self.conn
        orbit = vessel.orbit
        body = vessel.orbit.body
        flight = vessel.flight(body.orbital_reference_frame)
        ksc = conn.space_center

        inclination_limit = math.pi / 18 # 10 degrees
        if abs(orbit.inclination) > inclination_limit:
            raise Exception('Inclination is too large')

        # TODO check eccentricity

        rf = body.orbital_reference_frame
        direction = orbit.position_at(ksc.ut, rf)
        direction = direction / norm(direction)
        current_angle_from_prograde = arccos(direction[1]) # prograde = (0,1,0)

        #https://en.wikipedia.org/wiki/Hyperbolic_trajectory
        v_inf = body.orbit.speed
        v_inf_sq = v_inf * v_inf
        mu = body.gravitational_parameter
        r = orbit.semi_major_axis
        ecc = 1.0 + r * v_inf_sq / mu
        target_angle_from_prograde = arcsin(-1.0/ecc)

        true_anomaly = orbit.true_anomaly 
        true_anomaly += current_angle_from_prograde 
        true_anomaly -= target_angle_from_prograde
        ut = orbit.ut_at_true_anomaly(true_anomaly)

        v0 = math.sqrt(2.0 * mu / r + v_inf_sq)
        delta_v = v0 - orbit.orbital_speed_at(ut)

        node = control.add_node(ut, prograde=delta_v)

        next_body = body.orbit.body
        # all bodies in stock game with moons also have an atmosphere
        closest_approach = next_body.equatorial_radius + 0.75 * next_body.atmosphere_depth

        notify('raising periapsis')
        while node.orbit.next_orbit.periapsis < closest_approach:
            node.prograde -= 1
        notify('refining maneuver time')
        prev_periapsis = node.orbit.next_orbit.periapsis * 2
        while prev_periapsis > node.orbit.next_orbit.periapsis:
            prev_periapsis = node.orbit.next_orbit.periapsis
            node.ut += 1
        notify('raising periapsis')
        while node.orbit.next_orbit.periapsis < closest_approach:
            node.prograde -= 1
        
        self.execute_next_node()
        ksc.warp_to(ksc.ut + vessel.orbit.time_to_soi_change + 1)
