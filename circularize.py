import math

from common import wait, notify
from common.staging import StagingAware
from common.burn_time import BurnTime
from common.g_limit import GLimited

class Circularize(StagingAware, BurnTime, GLimited):
    def __init__(self, conn, target_heading=90.0):
        self.conn = conn
        self.vessel = conn.space_center.active_vessel
        self.target_heading = target_heading

        self.ap = self.vessel.auto_pilot
        self.should_stage = False
        self.running = True

    def circularization_delta_v(self):
        '''
        https://en.wikipedia.org/wiki/Vis-viva_equation
        '''
        mu = self.vessel.orbit.body.gravitational_parameter
        r = self.apoapsis()
        a1 = self.semi_major_axis()
        a2 = r
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        delta_v = v2 - v1
        return delta_v

    def connect_streams(self):
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.semi_major_axis = self.conn.add_stream(getattr, self.vessel.orbit, 'semi_major_axis')
        self.eccentricity = self.conn.add_stream(getattr, self.vessel.orbit, 'eccentricity')
        self.apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis')
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')

    def execute(self):
        if self.vessel.orbit.eccentricity < 0.1:
            return
        notify("Circularizing...")
        self.connect_streams()

        self.initial_apoapsis = self.apoapsis()

        self.vessel.control.throttle = 0.0
        self.ap.target_pitch_and_heading(0.0, self.target_heading)
        self.ap.engage()

        dv = self.circularization_delta_v()
        burn_time = self.burn_time(dv)
        ut = self.ut()
        apoapsis_ut = ut + self.vessel.orbit.time_to_apoapsis
        start_ut = apoapsis_ut - burn_time / 2
        end_ut = apoapsis_ut + burn_time / 2
        lead_time = 10.0 # seconds
        eccentricity = self.eccentricity()

        notify(f"dv: {dv}")
        notify(f"burn_time: {burn_time}")
        notify(f"initial_eccentricity: {eccentricity}")

        #TODO: point to apoapsis prograde before warp
        self.conn.space_center.warp_to(start_ut - lead_time)
        while self.running:
            self.check_staging()

            ut = self.ut()
            prev_eccentricity = eccentricity
            eccentricity = self.eccentricity()
            increasing = eccentricity > prev_eccentricity
            remaining_burn = self.circularization_delta_v()

            if ut < start_ut:
                pass
            elif ut < end_ut and remaining_burn > 50:
                self.vessel.control.throttle = 1.0
            elif not increasing:
                self.g_limit(0.5)
            else:
                self.vessel.control.throttle = 0.0
                self.running = False

            wait()
        notify(f"final_eccentricity: {eccentricity}")

