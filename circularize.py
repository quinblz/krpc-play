import math

from common import wait, notify
from common.staging import StagingAware
from common.g_limit import GLimited
from common.maneuver import Maneuver

class Circularize(StagingAware, GLimited, Maneuver):
    def __init__(self, conn, target_heading=90.0, **kwargs):
        super().__init__(conn, **kwargs)
        self.target_heading = target_heading

    def circularization_delta_v(self):
        '''
        https://en.wikipedia.org/wiki/Vis-viva_equation
        '''
        orbit = self.vessel.orbit
        mu = orbit.body.gravitational_parameter
        r = orbit.apoapsis
        a1 = orbit.semi_major_axis
        a2 = r
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        delta_v = v2 - v1
        return delta_v

    def execute(self):
        if self.vessel.orbit.eccentricity < 0.1:
            return
        notify("Circularizing...")

        self.initial_apoapsis = self.apoapsis()
        control = self.vessel.control
        control.throttle = 0.0

        # ap = self.vessel.auto_pilot
        # notify('pointing...')
        # ap.engage()
        # ap.target_pitch_and_heading(0.0, self.target_heading)
        # ap.wait()
        # notify('aligned...')

        delta_v = self.circularization_delta_v()
        burn_time = self.burn_time(delta_v)
        ut = self.ut()
        apoapsis_ut = ut + self.vessel.orbit.time_to_apoapsis
        # start_ut = apoapsis_ut - burn_time / 2
        # end_ut = apoapsis_ut + burn_time / 2
        # lead_time = 10.0 # seconds
        eccentricity = self.eccentricity()

        notify(f"delta_v: {delta_v}")
        notify(f"burn_time: {burn_time}")
        notify(f"initial_eccentricity: {eccentricity}")

        control.add_node(apoapsis_ut, prograde=delta_v)
        self.execute_next_node()
        notify(f"final_eccentricity: {self.eccentricity()}")

        #TODO: point to apoapsis prograde before warp
        # self.conn.space_center.warp_to(start_ut - lead_time)
        # while self.running:
        #     self.check_staging()

        #     ut = self.ut()
        #     prev_eccentricity = eccentricity
        #     eccentricity = self.eccentricity()
        #     increasing = eccentricity > prev_eccentricity
        #     remaining_burn = self.circularization_delta_v()

        #     if ut < start_ut:
        #         pass
        #     elif ut < end_ut and remaining_burn > 50 and not increasing:
        #         self.vessel.control.throttle = 1.0
        #     elif not increasing:
        #         self.g_limit(0.5)
        #     else:
        #         self.vessel.control.throttle = 0.0
        #         self.running = False

        #     wait()
        # notify(f"final_eccentricity: {eccentricity}")

