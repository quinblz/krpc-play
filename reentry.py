import time

from common import notify, wait
from common.maneuver import Maneuver

class CapsuleReentry(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def connect_streams(self):
        super().connect_streams()

    def execute(self):
        vessel = self.vessel
        ksc = self.conn.space_center
        orbit = vessel.orbit
        body = orbit.body
        control = vessel.control

        while max([part.decouple_stage for part in vessel.parts.all]) > -1:
            control.activate_next_stage()

        warp_radius = body.equatorial_radius + body.atmosphere_depth
        anom = orbit.true_anomaly_at_radius(warp_radius)
        ut = min(orbit.ut_at_true_anomaly(anom), orbit.ut_at_true_anomaly(-anom))
        ksc.warp_to(ut - 15)

        control.sas = True
        control.sas_mode = control.sas_mode.retrograde
