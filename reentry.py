import time

from common import notify, wait
from common.staging import StagingAware
from common.maneuver import Maneuver

class CapsuleReentry(StagingAware, Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def execute(self):
        vessel = self.vessel
        ksc = self.conn.space_center
        orbit = vessel.orbit
        body = orbit.body
        control = vessel.control

        atmosphere = body.atmosphere_depth
        warp_radius = body.equatorial_radius + atmosphere
        anom = orbit.true_anomaly_at_radius(warp_radius)
        ut = min(orbit.ut_at_true_anomaly(anom), orbit.ut_at_true_anomaly(-anom))
        ksc.warp_to(ut - 15)

        control.sas = True
        control.sas_mode = control.sas_mode.retrograde
        wait(5)
        control.throttle = 1.0
        self.setup_staging_callback()
        while self.running:
            self.check_staging()
            if self.apoapsis_altitude() < atmosphere:
                break
        control.throttle = 0

        # TODO: handle decoupling heat shields
        while max([part.decouple_stage for part in vessel.parts.all]) > -1:
            control.activate_next_stage()

        control.sas = True
        control.sas_mode = control.sas_mode.retrograde
