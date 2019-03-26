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
        vessel = self.vessel
        control = vessel.control
        control.sas = True
        body = vessel.orbit.body
        surface_gravity = body.surface_gravity
        r = body.equatorial_radius
        degrees_per_meter = 180 / r / math.pi
        
        telem = self.vessel.flight(body.reference_frame)
        vertical_speed = self.conn.add_stream(getattr, telem, 'vertical_speed')
        speed = self.conn.add_stream(getattr, telem, 'speed')

        if not self.TWR():
            control.activate_next_stage()

        throttle_control = PID(1.0, 0.0, 1.5)
        throttle_error = []
        # target_altitude = self.surface_altitude()
        target_altitude = self.surface_altitude()

        def update_throttle():
            p_err = target_altitude - self.surface_altitude()
            # acc = surface_gravity - 1.5 * vertical_speed() + p_err
            acc = surface_gravity + throttle_control(-p_err)
            throttle = acc / self.TWR()
            throttle_error.append(p_err)
            control.throttle = throttle
            notify(self.surface_altitude())

        while not self.abort():
            if self.brakes():
                target_altitude = 100
            else:
                target_altitude = 110
            update_throttle()
            wait()


        control.throttle = 0

        from scipy.optimize import least_squares
        error = np.array(throttle_error)
        t = np.arange(len(error))
        guess = np.ones(3)
        # res = least_squares(model, guess, args=(t, error))
        # x = res.x
        # fit = model(x, t, 0.0)
        # plt.plot(t, fit, label=f'{x}')
        plt.plot(t, np.zeros_like(t), label='zero')
        plt.plot(t, np.log(np.abs(error) + 1.0), label='log')
        plt.plot(t, error, label='raw')
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