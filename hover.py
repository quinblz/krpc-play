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
        
        telem = self.vessel.flight(body.reference_frame)
        vertical_speed = self.conn.add_stream(getattr, telem, 'vertical_speed')
        speed = self.conn.add_stream(getattr, telem, 'speed')

        if not self.TWR():
            control.activate_next_stage()

        throttle_control = PID(1.0, 0.0, 1.5)
        throttle_error = []
        def update_throttle():
            err = target.altitude - self.surface_altitude()
            # acc = surface_gravity - 1.5 * vertical_speed() + p_err
            acc = surface_gravity + throttle_control(-err)
            throttle = acc / self.TWR()
            throttle_error.append(err)
            return throttle

        lat_control = PID(0.2, 0.0, 0.6)
        lon_control = PID(0.2, 0.0, 0.6)
        lat_control.output_limits = -1.0, 1.0
        lon_control.output_limits = -1.0, 1.0
        lat_error = []
        lon_error = []
        line = self.conn.drawing.add_direction((1,0,0), rf)
        def update_direction():
            scale = 0.2
            lat_err = (target.latitude - self.latitude()) / degrees_per_meter
            lon_err = (target.longitude - self.longitude()) / degrees_per_meter
            lat_out = lat_control(lat_err) * scale
            lon_out = lon_control(lon_err) * scale
            lat_error.append(lat_err)
            lon_error.append(lon_err)
            vec = np.array([1.0, -lat_out, -lon_out])
            line.end = (0.0, lat_err, lon_err)
            notify(lat_err, lon_err, vec)
            return vec

        target.altitude = 100.0
        target.latitude = self.latitude()
        target.longitude = self.longitude()
        while not self.abort():
            if self.brakes():
                target.latitude = lat1
                target.longitude = lon1
            else:
                target.latitude = lat0
                target.longitude = lon0
            d = update_direction()
            ap.target_direction = d
            control.throttle = update_throttle() * np.linalg.norm(d)
        control.throttle = 0

        from scipy.optimize import least_squares
        lat_error = np.array(lat_error)
        lon_error = np.array(lon_error)
        t = np.arange(len(lat_error))
        # guess = np.ones(3)
        # res = least_squares(model, guess, args=(t, error))
        # x = res.x
        # fit = model(x, t, 0.0)
        # plt.plot(t, fit, label=f'{x}')
        plt.plot(t, np.zeros_like(t), label='zero')
        plt.plot(t, np.log(np.abs(lat_error) + 1.0), label='log lat')
        plt.plot(t, lat_error, label='raw lat')
        plt.plot(t, np.log(np.abs(lon_error) + 1.0), label='log lon')
        plt.plot(t, lon_error, label='raw lon')
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