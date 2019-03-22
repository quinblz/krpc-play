import time
from common import (
    clamp,
    countdown_to,
    wait,
)
from common.g_limit import GLimited
from common.staging import StagingAware
from common.maneuver import Maneuver

class Launch(GLimited, StagingAware, Maneuver):
    def __init__(self, conn, target_apoapsis=1e5, target_heading=90.0, **kwargs):
        super().__init__(conn, **kwargs)
        self.conn = conn
        self.vessel = conn.space_center.active_vessel
        self.target_heading = target_heading
        self.target_apoapsis = target_apoapsis
        
        self.ap = self.vessel.auto_pilot
        self.ap.reference_frame = self.vessel.surface_reference_frame
        self.running = True
        self.should_stage = False

    def gravity_turn(self):
        start_roll = 250
        end_roll = 45000
        completion = clamp((self.mean_altitude() - start_roll) / (end_roll - start_roll))
        target_pitch = 90 * (1.0 - completion)
        if abs(target_pitch - self.ap.target_pitch) > 0.5:
            self.ap.target_pitch_and_heading(target_pitch, self.target_heading)

    def execute(self):
        if self.apoapsis_altitude() > self.target_apoapsis:
            return

        self.ap.engage()
        self.ap.target_pitch_and_heading(90.0, self.target_heading)
        self.vessel.control.throttle = 1.0

        if(self.vessel.situation == self.conn.space_center.VesselSituation.pre_launch):
            self.ap.target_pitch_and_heading(90, self.target_heading)
            countdown_to("LAUNCH!")
            self.check_staging(force=True)
        else:
            self.setup_staging_callback()

        while self.running:
            self.check_staging()

            if self.surface_altitude() < 250:
                self.g_limit(2.2)
            elif self.apoapsis_altitude() < 0.9 * self.target_apoapsis:
                self.g_limit(2.2)
                self.gravity_turn()
            elif self.apoapsis_altitude() < self.target_apoapsis:
                self.g_limit(1.0)
            else:
                self.vessel.control.throttle = 0.0
                self.running = False
            wait()