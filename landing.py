import math

from common.maneuver import Maneuver
from common import notify, wait

class Land(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def connect_streams(self):
        super().connect_streams()
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')

    def warp_to_soi(self):
        notify('Warping to target sphere of influence')
        vessel = self.vessel
        orbit = vessel.orbit
        while orbit.periapsis_altitude > 0 and orbit.next_orbit and orbit.body != self.target:
            self.conn.space_center.warp_to(self.ut() + orbit.time_to_soi_change + 1)
            orbit = vessel.orbit

        
    def lower_periapsis(self):
        # lower periapsis to safe level in atmosphere
        pass

    def reentry_burn(self):
        # warp to atmospheric depth and burn until speed is below critical level
        pass

    def suicide_burn(self, telem, buffer_altitude=0):
        vessel = self.vessel
        effective_radius = telem.mean_altitude / 2 + vessel.orbit.body.equatorial_radius
        mu = vessel.orbit.body.gravitational_parameter
        a = - mu / (effective_radius * effective_radius)
        v = self.vertical_speed()
        y0 = telem.mean_altitude - buffer_altitude
        delta_t = (-v - math.sqrt(v*v - 2 * a * y0)) / a
        delta_v = v + delta_t * a
        return delta_t, delta_v
    
    def execute(self):
        self.connect_streams()
        self.warp_to_soi()

        notify('Landing')
        vessel = self.vessel

        if vessel.orbit.periapsis_altitude > 0:
            self.lower_periapsis()

        if vessel.orbit.body.has_atmosphere:
            self.reentry_burn()

        telem = vessel.flight(vessel.orbit.body.reference_frame)
        self.vertical_speed = self.conn.add_stream(getattr, telem, 'vertical_speed')
        vessel.control.speed_mode = vessel.control.speed_mode.surface
        rf = vessel.orbit.body.reference_frame
        ap = vessel.auto_pilot
        ap.sas = True
        ap.sas_mode = ap.sas_mode.retrograde

        vessel.control.legs = True
        vessel.control.lights = True
        vessel.control.brakes = True
        vessel.control.solar_panels = False

        delta_t, delta_v = self.suicide_burn(telem)
        burn_time = self.burn_time(delta_v)

        notify('Suicide Burn!')
        vessel.control.throttle = 1.0
        while(self.vertical_speed() < -100):
            wait()
        vessel.control.throttle = 0.0

        print('pause')

