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
        lat1 = lat0 + 100 * degrees_per_meter
        lon0 = self.longitude()
        lon1 = lon0 + 20 * degrees_per_meter
        target = lambda : None

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

        target_control = PID(0.2, 0.0, 0.6)
        perpendicular_control = PID(0.2, 0.0, 0.6)
        target_control.output_limits = -1.0, 1.0
        perpendicular_control.output_limits = -1.0, 1.0
        target_error = []
        perpendicular_error = []
        white_line = self.conn.drawing.add_direction((1,0,0), rf)
        green_line = self.conn.drawing.add_direction((1,0,0), rf)
        green_line.color = (0, 255, 0)
        red_line = self.conn.drawing.add_direction((1,0,0), rf)
        red_line.color = (255, 0, 0)
        mag_line = self.conn.drawing.add_direction((1,0,0), rf)
        mag_line.color = (255, 0, 255)

        def update_direction():
            target.pointer = ksc.transform_position(target.position, target.reference_frame, rf)
            target.distance = np.linalg.norm(target.pointer)
            target.relative_velocity = ksc.transform_velocity(target.position, (0,0,0), target.reference_frame, rf)

            p = np.array(target.pointer)
            p[0] = 0
            p_mag = np.linalg.norm(p)
            p_dir = p / p_mag
            p_perp = np.array([p_dir[0], -p_dir[2], p_dir[1]])

            v = np.array(target.relative_velocity)
            v[0] = 0
            v_mag = np.linalg.norm(v)
            v_dir = v / v_mag
            v_perp = np.array([v_dir[0], -v_dir[2], v_dir[1]])

            p_forward = np.dot(v_dir, p)
            p_side = np.dot(v_perp, p)

            v_torwards = np.dot(p_dir, v)
            v_side = np.dot(p_perp, v)

            vec = np.array([1.0, 0.0, 0.0]) # Point up if not correcting

            target_output = target_control(p_mag)
            vec -= min(0.2, v_mag) * target_output * p_dir

            perpendicular_output = perpendicular_control(p_side)
            vec += min(0.2, v_mag) * perpendicular_output * p_perp

            notify(p_mag, v_mag, (target_output, perpendicular_output))
            white_line.end = target.pointer
            green_line.end = 5.0 * p_dir
            red_line.end = 5.0 * p_perp
            mag_line.end = -10.0 * v_dir

            target_error.append(p_mag)
            perpendicular_error.append(p_side)

            return vec

        target.altitude = 50.0
        target.latitude = self.latitude()
        target.longitude = self.longitude()
        while not self.abort():
            if self.brakes():
                target.latitude = lat1
                target.longitude = lon1
                target.position = body.position_at_altitude(
                    target.latitude, target.longitude,
                    self.mean_altitude(), body.reference_frame)
                target.reference_frame = body.reference_frame
            else:
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
        perpendicular_error = np.array(perpendicular_error)
        target_error = np.array(target_error)
        t = np.arange(len(perpendicular_error))
        # guess = np.ones(3)
        # res = least_squares(model, guess, args=(t, error))
        # x = res.x
        # fit = model(x, t, 0.0)
        # plt.plot(t, fit, label=f'{x}')
        plt.plot(t, np.zeros_like(t), label='zero')
        plt.plot(t, np.log(np.abs(perpendicular_error) + 1.0), label='log perp')
        plt.plot(t, perpendicular_error, label='raw perp')
        plt.plot(t, np.log(np.abs(target_error) + 1.0), label='log target')
        plt.plot(t, target_error, label='raw target')
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