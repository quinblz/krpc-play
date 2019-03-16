import time
from common import (
    clamp,
    countdown_to,
    wait,
)
from common.g_limit import GLimited
from common.staging import StagingAware

class Launch(GLimited,StagingAware):
    def __init__(self, conn, vessel, target_apoapsis=1e5, target_heading=90.0):
        self.conn = conn
        self.vessel = conn.space_center.active_vessel
        self.target_heading = target_heading
        self.target_apoapsis = target_apoapsis
        
        self.ap = self.vessel.auto_pilot
        self.running = True
        self.should_stage = False

    def gravity_turn(self):
        start_roll = 250
        end_roll = 45000
        completion = clamp((self.altitude() - start_roll) / (end_roll - start_roll))
        target_pitch = 90 * (1.0 - completion)
        if abs(target_pitch - self.ap.target_pitch) > 0.5:
            self.ap.target_pitch_and_heading(target_pitch, self.target_heading)

    def create_streams(self):
        self.altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'mean_altitude')
        self.apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')

    def execute(self):
        self.create_streams()
            
        self.vessel.control.throttle = 1.0
        self.ap.engage()

        if(self.vessel.situation == self.conn.space_center.VesselSituation.pre_launch):
            self.ap.target_pitch_and_heading(90, self.target_heading)
            countdown_to("LAUNCH!")
            self.check_staging(force=True)
        else:
            self.setup_staging_callback()

        while self.running:
            self.check_staging()

            if self.apoapsis() < 0.9 * self.target_apoapsis:
                self.g_limit(2.2)
                self.gravity_turn()
            elif self.apoapsis() < self.target_apoapsis:
                self.g_limit(1.0)
            else:
                self.vessel.control.throttle = 0.0
                self.running = False
            wait()