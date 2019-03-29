import math
import numpy as np
import time

import scipy as sp
import matplotlib.pyplot as plt

from common import notify, wait
from common.g_limit import GLimited
from common.maneuver import Maneuver
from common.pid import PID

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
        control = vessel.control
        control.rcs = True
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
            control.activate_next_stage()

        pid = [
            PID(1.0, 0.0, 1.5), # throttole
            PID(0.2, 0.0, 0.6), # N/S
            PID(0.2, 0.0, 0.6), # E/W
        ]
        pid[1].output_limits = -1.0, 1.0
        pid[2].output_limits = -1.0, 1.0
        error = [[],[],[]]

        throttle_error = error[0]
        def update_throttle():
            err = target.altitude - self.surface_altitude()
            # acc = surface_gravity - 1.5 * vertical_speed() + p_err
            acc = surface_gravity + pid[0](-err)
            throttle = acc / self.TWR()
            throttle_error.append(err)
            return throttle

        def update_direction():
            target.pointer = ksc.transform_position(target.position, target.reference_frame, rf)
            target.distance = np.linalg.norm(target.pointer)
            target.relative_velocity = -np.array(ksc.transform_velocity(target.position, (0,0,0), target.reference_frame, rf))
            target.speed = np.linalg.norm(target.relative_velocity)

            p = np.array(target.pointer)
            scale = 0.2
            vec = [1.0, -scale * pid[1](p[1]), -scale * pid[2](p[2])]
            error[1].append(p[1])
            error[2].append(p[2])
            notify(target.distance, target.speed)
            return vec

        target.altitude = 50.0
        target.latitude = self.latitude()
        target.longitude = self.longitude()
        while not self.abort():
            if self.brakes():
                target.altitude = 100.0
                target.latitude = lat1
                target.longitude = lon1
                target.position = body.position_at_altitude(
                    target.latitude, target.longitude,
                    self.mean_altitude(), body.reference_frame)
                target.reference_frame = body.reference_frame
            else:
                target.altitude = 50
                target.latitude = lat0
                target.longitude = lon0
                target.position = body.position_at_altitude(
                    target.latitude, target.longitude,
                    self.mean_altitude(), body.reference_frame)
                target.reference_frame = body.reference_frame
            d = update_direction()
            ap.target_direction = d
            control.throttle = update_throttle() * np.linalg.norm(d)
            wait(0.01)
        control.throttle = 0

        from scipy.optimize import least_squares
        err1 = np.array(error[1])
        err2 = np.array(error[2])
        t = np.arange(len(err1))
        # guess = np.ones(3)
        # res = least_squares(model, guess, args=(t, error))
        # x = res.x
        # fit = model(x, t, 0.0)
        # plt.plot(t, fit, label=f'{x}')
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


def model(x, t, y):
    return x[0] * np.exp(-x[1] * t) * np.cos(x[2] * t) - y

if __name__ == "__main__":
    import krpc
    conn = krpc.connect()
    Hover(conn).execute()