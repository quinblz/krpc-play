import math
import time

from common.maneuver import Maneuver
from common.pid import PID
from common import notify, wait

class Land(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def connect_streams(self):
        super().connect_streams()
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.surface_altitude = self.conn.add_stream(getattr, self.vessel.flight(), 'surface_altitude')
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')
        self.situation = self.conn.add_stream(getattr, self.vessel, 'situation')

        telem = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.vertical_speed = self.conn.add_stream(getattr, telem, 'vertical_speed')
        self.speed = self.conn.add_stream(getattr, telem, 'speed')

        self.surface_gravity = self.vessel.orbit.body.surface_gravity

    def warp_to_soi(self):
        vessel = self.vessel
        orbit = vessel.orbit
        while orbit.periapsis_altitude > 0 and orbit.next_orbit and orbit.body != self.target:
            notify('Warping to next sphere of influence...')
            self.conn.space_center.warp_to(self.ut() + orbit.time_to_soi_change + 1)
            orbit = vessel.orbit
        
    def lower_periapsis(self):
        # lower periapsis to safe level in atmosphere
        pass

    def reentry_burn(self):
        # warp to atmospheric depth and burn until speed is below critical level
        pass

    def estimate_suicide_burn(self):
        vessel = self.vessel
        orbit = vessel.orbit
        buffer = 100
        r = vessel.orbit.body.equatorial_radius + buffer
        d = self.surface_altitude() - buffer
        mu = vessel.orbit.body.gravitational_parameter
        g = mu / (r * r)
        v = self.speed()
        delta_v = v + math.sqrt(2 * mu * ( 1/r - 1/(r+d)))
        delta_t = self.burn_time(delta_v, against_g=g)
        #TTI = 2 * d / (delta_v + v)
        #g_avg = 0.5 * g + 0.5 * mu / ((d+r)*(d+r))
        #TTI = (v + math.sqrt(v*v + 2*g_avg*d)) / g_avg
        #TTI = (delta_v - v) / g_avg

        anom = orbit.true_anomaly_at_radius(r + buffer)
        ut = min(orbit.ut_at_true_anomaly(anom), orbit.ut_at_true_anomaly(-anom))

        notify(ut - self.ut(), delta_v, delta_t)
        return ut, delta_v, delta_t

    def slow_down(self, vessel):
        notify('Slowing down...')
        g = self.surface_gravity
        ut, delta_v, delta_t = self.estimate_suicide_burn()
        self.conn.space_center.warp_to(ut - 0.5 * delta_t)

        vessel.control.throttle = 1.0
        effective_twr = self.TWR() - self.surface_gravity
        notify(f'surface_altitude:{self.surface_altitude()}, vertical_speed:{self.vertical_speed()}, effective_twr:{effective_twr}')
        while -self.vertical_speed() > 10 * effective_twr:
            wait()
        vessel.control.throttle = 0.0
        notify(f'surface_altitude:{self.surface_altitude()}, vertical_speed:{self.vertical_speed()}')

    def stopping_distance(self):
        v = self.speed()
        return 0.5 * v * v / (0.95 * self.TWR() - self.surface_gravity)
    
    def controlled_descent(self):
        LANDED = self.conn.space_center.VesselSituation.landed
        control = self.vessel.control
        control.trottle = 0
        control.brakes = True
        control.legs = True
        control.lights = True
        control.solar_panels = False
        control.sas = True
        control.sas_mode = control.sas_mode.retrograde
        
        notify('Waiting for landing burn...')
        buffer = 25
        while self.surface_altitude() > self.stopping_distance() + buffer:
            wait()

        landing_speed = 0.5
        v_target = lambda: 0.2 * self.surface_altitude() + landing_speed
        base_throttle = lambda: self.surface_gravity / self.TWR()

        effective_twr = self.TWR() - self.surface_gravity
        control.throttle = 1.0
        notify('Landing burn!')
        while self.speed() > 10 * effective_twr:
            wait()

        pid = PID(0.1, 0.0, 0.0,
            sample_time=wait.interval,
            output_limits=(-1.0,1.0))
        notify('Entering controlled descent...')
        while self.situation() != LANDED:
            altitude = self.surface_altitude()
            base = base_throttle()
            target = v_target() 
            actual = self.speed()
            pid_input = target - actual
            pid_output = pid(pid_input)
            notify(f'altitude:{altitude:.2f}, target:{target:.2f}, actual:{actual:.2f}, input:{pid_input:.2f}, output:{pid_output:.2f}')
            control.throttle = base + pid_output
            if self.surface_altitude() < 25:
                control.sas_mode = control.sas_mode.stability_assist
            wait()
        control.throttle = 0
    
    def execute(self):
        self.warp_to_soi()
        self.connect_streams()
        if self.situation() == self.conn.space_center.VesselSituation.landed:
            return
        vessel = self.vessel

        vessel.control.speed_mode = vessel.control.speed_mode.surface
        ap = vessel.auto_pilot
        ap.sas = True
        ap.sas_mode = ap.sas_mode.retrograde

        vessel.control.solar_panels = False

        if vessel.orbit.periapsis_altitude > 0:
            self.lower_periaps
        if vessel.orbit.body.has_atmosphere:
            self.reentry_burn()
        if self.surface_altitude() > 20000:
            self.slow_down(vessel)

        self.controlled_descent()

        notify('Landed?')