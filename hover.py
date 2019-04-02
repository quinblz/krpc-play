import math
import numpy as np
import time

import scipy as sp
import matplotlib.pyplot as plt

from common import notify, wait
from common.g_limit import GLimited
from common.maneuver import Maneuver
from common.pid import PID, CascadeControl

class Hover(GLimited, Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def execute(self):
        ksc = self.conn.space_center
        vessel = self.vessel
        rf = vessel.surface_reference_frame # x: up, y: north, z: east
        ap = vessel.auto_pilot
        ap.engage()
        ap.reference_frame = rf
        vessel.control.rcs = True
        body = vessel.orbit.body
        surface_gravity = body.surface_gravity
        r = body.equatorial_radius
        degrees_per_meter = 180 / r / math.pi

        lat0 = self.latitude() + 50 * degrees_per_meter
        lat1 = lat0 + 10 * degrees_per_meter
        lon0 = self.longitude()
        lon1 = lon0 + 20 * degrees_per_meter
        target = lambda : None

        if not self.TWR():
            vessel.control.activate_next_stage()


        def create_lateral_control():
            Kp = 0.2
            Kd = 3.0 * Kp
            lateral_acc = 0.2 * surface_gravity
            control = PID(
                Kp=Kp, Kd=Kd,
                output_limits=(-lateral_acc, lateral_acc)
            )
            return control
        controllers = [
            PID(1.0, 0.0, 1.5), # up/down
            create_lateral_control(), # N/S
            create_lateral_control(), # E/W
        ]
        error = [[],[],[]]

        def update_throttle():
            err = target.altitude - self.surface_altitude()
            acc = surface_gravity + controllers[0](-err)
            throttle = acc / self.TWR()
            error[0].append(err)
            return throttle

        def force_vector():
            p = -np.array(target.pointer)
            scale = min(1.0, max(target.distance, 2.0 * target.speed))
            vec = np.array([surface_gravity, scale * controllers[1](p[1]), scale * controllers[2](p[2])])
            error[1].append(p[1])
            error[2].append(p[2])
            return vec

        target.altitude = 50.0
        target.latitude = self.latitude()
        target.longitude = self.longitude()
        while not self.abort():
            if self.brakes():
                target.altitude = 100.0
                target.latitude = lat1
                target.longitude = lon1
            else:
                target.altitude = 50
                target.latitude = lat0
                target.longitude = lon0
            
            target.position = body.position_at_altitude(
                target.latitude, target.longitude,
                self.mean_altitude(), body.reference_frame)
            target.reference_frame = body.reference_frame
            target.pointer = ksc.transform_position(target.position, target.reference_frame, rf)
            target.distance = np.linalg.norm(target.pointer)
            target.relative_velocity = -np.array(ksc.transform_velocity(target.position, (0,0,0), target.reference_frame, rf))
            target.speed = np.linalg.norm(target.relative_velocity)
            
            notify(target.distance, target.speed)

            F = force_vector()
            ap.target_direction = F
            scaled_force = np.linalg.norm(F) / surface_gravity
            vessel.control.throttle = update_throttle() * scaled_force
            wait(0.01)
        vessel.control.throttle = 0

        err1 = np.array(error[1])
        err2 = np.array(error[2])
        t = np.arange(len(err1))
        plt.plot(t, np.zeros_like(t), label='zero')
        plt.plot(t, np.log(np.abs(err1) + 1.0), label='log err1')
        plt.plot(t, err1, label='raw err1')
        plt.plot(t, np.log(np.abs(err2) + 1.0), label='log err2')
        plt.plot(t, err2, label='raw err2')
        plt.xlabel('time')
        plt.ylabel('error')
        plt.legend()
        plt.show()

        wait()

if __name__ == "__main__":
    import krpc
    conn = krpc.connect()
    Hover(conn).execute()