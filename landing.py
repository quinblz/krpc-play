import math
import numpy as np
import time

from common.maneuver import Maneuver
from common.pid import PID
from common import notify, wait, normed


def cached(fn):
    cache = {}
    def wrapped(*args):
        if args not in cache:
            cache[args] = fn(*args)
        return cache[args]
    wrapped.inner = fn
    wrapped.cache = cache
    return wrapped

class Land(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)

    def connect_streams(self):
        telem = self.vessel.flight(self.vessel.orbit.body.reference_frame)
        self.vertical_speed = self.conn.add_stream(getattr, telem, 'vertical_speed')
        self.speed = self.conn.add_stream(getattr, telem, 'speed')

        self.surface_gravity = self.vessel.orbit.body.surface_gravity

    def warp_to_soi(self):
        vessel = self.vessel
        orbit = vessel.orbit
        ksc = self.conn.space_center
        while orbit.time_to_soi_change < orbit.time_to_periapsis:
            notify('Warping to next sphere of influence...')
            ksc.warp_to(ksc.ut + orbit.time_to_soi_change + 1)
            orbit = vessel.orbit
        
    def lower_periapsis(self):
        # TODO: handle atmosphere
        if self.vessel.orbit.body.has_atmosphere:
            raise Exception('Unable to lower periapsis on body with atmosphere')
        node_time = self.ut() + self.vessel.orbit.time_to_periapsis
        node = self.vessel.control.add_node(node_time)
        target_periapsis = 0.25 * self.vessel.orbit.body.equatorial_radius
        while node.orbit.periapsis > target_periapsis:
            node.prograde -= 1.0
        self.execute_next_node()

    def reentry_burn(self):
        # warp to atmospheric depth and burn until speed is below critical level
        pass

    def sample_surface_height(self):
        orbit = self.vessel.orbit
        body = orbit.body
        rf = body.reference_frame
        ut = self.ut()
        samples = 50
        times = np.linspace(ut, ut + orbit.time_to_periapsis, samples)
        def sample(time):
            pos = orbit.position_at(time, rf)
            lat = body.latitude_at_position(pos, rf)
            lon = body.longitude_at_position(pos, rf)
            return body.surface_height(lat, lon)
        samples = [sample(time) for time in times]
        return max(samples)

    def estimate_suicide_burn(self):
        vessel = self.vessel
        orbit = vessel.orbit
        body = orbit.body
        buffer = 100
        ut = self.ut()
        max_height = self.sample_surface_height() * 1.1
        r = body.equatorial_radius + max_height + buffer
        d = self.surface_altitude() - buffer
        mu = body.gravitational_parameter
        g = mu / (r * r)
        v = self.speed()
        # https://en.wikipedia.org/wiki/Equations_for_a_falling_body
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
        
        vessel = self.vessel
        control = vessel.control
        rf = vessel.orbit.body.reference_frame
        flight = vessel.flight(rf)
        _surface_velocity = self.conn.add_stream(getattr, flight, 'velocity')
        surface_velocity = lambda: -np.array(_surface_velocity())
        ap = vessel.auto_pilot
        ap.engage()
        ap.reference_frame = rf
        ap.target_direction = surface_velocity()
        ap.wait()

        g = self.surface_gravity
        ut, delta_v, delta_t = self.estimate_suicide_burn()

        self.conn.space_center.warp_to(ut - 0.5 * delta_t)
        
        control.throttle = 1.0
        effective_twr = self.TWR() - self.surface_gravity
        notify(f'surface_altitude:{self.surface_altitude()}, vertical_speed:{self.vertical_speed()}, effective_twr:{effective_twr}')
        while -self.vertical_speed() > 10 * effective_twr:
            ap.target_direction = surface_velocity()
            wait()
        control.throttle = 0.0
        notify(f'surface_altitude:{self.surface_altitude()}, vertical_speed:{self.vertical_speed()}')

    def find_safe_landing(self):
        body = self.vessel.orbit.body
        r = body.equatorial_radius
        degrees_per_meter = 180 / r / math.pi
        epsilon = degrees_per_meter

        @cached
        def evaluate(lat, lon):
            height = body.surface_height(lat, lon)
            lat_delta = body.surface_height(lat+epsilon, lon) - height
            lon_delta = body.surface_height(lat, lon+epsilon) - height
            flatness = lat_delta * lat_delta + lon_delta * lon_delta
            return flatness
        
        def scan(lat, lon, radius):
            delta = radius * degrees_per_meter
            samples = 10
            lats = np.linspace(lat-delta, lat+delta, samples)
            lons = np.linspace(lon-delta, lon+delta, samples)
            target = None
            min_flatness = float('inf')
            for lat in lats:
                for lon in lons:
                    flatness = evaluate(lat, lon)
                    if flatness < min_flatness:
                        target = (lat, lon)
                        min_flatness = flatness
            return target

        granularity = [500, 100] # scan radius in meters
        lat = self.latitude()
        lon = self.longitude()
        for radius in granularity:
            lat, lon = scan(lat, lon, radius)
        return lat, lon

    def stopping_distance(self):
        v = self.speed()
        return 0.5 * v * v / (0.95 * self.TWR() - self.surface_gravity)
    
    def controlled_descent(self, target=None):
        LANDED = self.conn.space_center.VesselSituation.landed
        SPLASHED = self.conn.space_center.VesselSituation.splashed
        vessel = self.vessel
        control = vessel.control
        control.trottle = 0
        control.brakes = True
        control.gear = True
        control.lights = True
        control.solar_panels = False
        control.sas = False

        lat, lon = target
        # rf = vessel.surface_reference_frame
        rf = vessel.orbit.body.reference_frame
        flight = vessel.flight(rf)
        _pos = self.conn.add_stream(getattr, flight, 'center_of_mass')
        pos = lambda: np.array(_pos())
        target_pos = lambda: np.array(vessel.orbit.body.surface_position(lat, lon, rf))
        _surface_velocity = self.conn.add_stream(getattr, flight, 'velocity')
        surface_velocity = lambda: -np.array(_surface_velocity())
        ap = vessel.auto_pilot
        ap.engage()
        ap.reference_frame = rf
        ap.target_direction = surface_velocity()

        landing_speed = 0.5
        v_target = lambda: 0.2 * self.surface_altitude() + landing_speed
        base_throttle = lambda: self.surface_gravity / self.TWR()
        effective_twr = self.TWR() - self.surface_gravity

        notify('Waiting for landing burn...')
        while self.surface_altitude() > self.stopping_distance() + 2.0 * wait.interval * self.speed():
            ap.target_direction = surface_velocity()
            wait()

        control.throttle = 1.0
        notify('Landing burn!')
        notify(f'altitude: {self.surface_altitude()}')
        while self.speed() > 5 * effective_twr:
            d = surface_velocity()
            ap.target_direction = d / np.linalg.norm(d)
            wait()

        notify(f'effective_twr: {effective_twr}')
        pid = PID(0.2, 0.0, 0.0,
            sample_time=wait.interval,
            output_limits=(-1.0,1.0))
        notify('Entering controlled descent...')
        while self.situation() not in [LANDED, SPLASHED]:
            altitude = self.surface_altitude()
            base = base_throttle()
            target = v_target() 
            actual = -self.vertical_speed()
            pid_input = target - actual
            pid_output = pid(pid_input)
            # notify({
            #     'altitude': altitude,
            #     'target': target,
            #     'actual': actual,
            #     'input': pid_input,
            #     'output': pid_output,
            #     'pos': pos(),
            #     'target_pos', target_pos,
            # })
            notify(f'altitude:{altitude:.2f}, target:{target:.2f}, actual:{actual:.2f}, input:{pid_input:.2f}, output:{pid_output:.2f}, pos: {pos()}, target: {target_pos()}')
            control.throttle = base + pid_output
            if self.surface_altitude() < 25:
                ap.target_direction = pos()
            else:
                d = surface_velocity()
                ap.target_direction = d
            wait()
        control.throttle = 0
        ap.disengage()
        notify('done')
    
    def execute(self):
        self.warp_to_soi()
        self.connect_streams()
        if self.situation() == self.conn.space_center.VesselSituation.landed:
            return
        vessel = self.vessel
        control = vessel.control

        vessel.control.solar_panels = False

        if self.periapsis_altitude() > 0:
            self.lower_periapsis()
        if vessel.orbit.body.has_atmosphere:
            self.reentry_burn()
        if self.surface_altitude() > 20000:
            self.slow_down(vessel)

        start = time.perf_counter()
        notify('scanning for landing site...')
        target = self.find_safe_landing()
        notify(f'scan took {time.perf_counter() - start}')
        notify(f'landing at {target}')
        self.controlled_descent(target)

        vessel.auto_pilot.disengage()
        control.sas = True
        control.sas_mode = control.sas_mode.stability_assist

        notify('Landed?')

        
if __name__ == "__main__":
    import krpc
    conn = krpc.connect()
    Land(conn).execute()