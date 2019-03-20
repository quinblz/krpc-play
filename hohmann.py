import math
import numpy
from numpy.linalg import norm
from common import clamp_radians, notify
from common.maneuver import Maneuver

def orbital_progress(orbit, ut=None):
    '''
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    '''
    lan = orbit.longitude_of_ascending_node
    arg_p = orbit.argument_of_periapsis
    if ut:
        ma = orbit.mean_anomaly_at_ut(ut)
    else:
        ma = orbit.mean_anomaly
    return clamp_radians(lan + arg_p + ma)

class HohmannTransfer(Maneuver):
    def __init__(self, conn, **kwargs):
        super().__init__(conn, **kwargs)
        ksc = conn.space_center
        if not self.target:
            raise ValueError("No target specified")

    def transfer_phase_angle(self):
        '''
        returns the relative phase angle for a hohmann transfer
        '''
        v_sma = self.vessel.orbit.semi_major_axis
        t_sma = self.target.orbit.semi_major_axis
        transfer_sma=(v_sma + t_sma) / 2
        # the proportion target orbit that goes by during the half period of transfer orbit
        phase_proportion = 1/(2*math.sqrt(math.pow(t_sma,3)/math.pow(transfer_sma,3)))
        phase_angle = 2 * math.pi * (1.0 - phase_proportion) 
        return phase_angle

    def mean_phase_rate(self):
        '''
        returns the rate of change of the phase angle between vessel and target
        '''
        vessel_rate = 2 * math.pi / self.vessel.orbit.period
        target_rate = 2 * math.pi / self.target.orbit.period
        return vessel_rate - target_rate

    def phase_angle(self, ut=None):
        v = orbital_progress(self.vessel.orbit, ut)
        t = orbital_progress(self.target.orbit, ut)
        return clamp_radians(t - v + math.pi)

    def transfer_delta_v(self, ut):
        '''
        returns the delta v required for a hohman transfer performed at time ut
        https://en.wikipedia.org/wiki/Vis-viva_equation
        '''
        mu = self.vessel.orbit.body.gravitational_parameter
        r = self.vessel.orbit.radius_at(ut)
        a1 = self.semi_major_axis()
        a2 = (self.vessel.orbit.apoapsis + self.target.orbit.apoapsis) / 2.0
        v1 = math.sqrt(mu*((2./r)-(1./a1)))
        v2 = math.sqrt(mu*((2./r)-(1./a2)))
        delta_v = v2 - v1
        return delta_v

    def refine_transfer(self, transfer_angle):
        transfer_time = self.ut()
        def error(ut):
            err = clamp_radians(self.phase_angle(ut) - transfer_angle)
            print(ut, err)
            return err
        def refine(ut, epsilon):
            previous_error = 2 * math.pi
            current_error = error(ut)
            while(current_error < previous_error):
                ut += epsilon
                previous_error = current_error
                current_error = error(ut)
            return ut
        transfer_time = refine(transfer_time, 10)
        transfer_time -= 10
        transfer_time = refine(transfer_time, 0.1)
        return transfer_time - 0.1

        
    def connect_streams(self):
        self.ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        self.semi_major_axis = self.conn.add_stream(getattr, self.vessel.orbit, 'semi_major_axis')
        self.eccentricity = self.conn.add_stream(getattr, self.vessel.orbit, 'eccentricity')
        self.apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis')
        self.mass = self.conn.add_stream(getattr, self.vessel, 'mass')
        self.available_thrust = self.conn.add_stream(getattr, self.vessel, 'available_thrust')

    def execute(self):
        if self.vessel.orbit.relative_inclination(self.target.orbit) > 0.01:
            raise Exception("Inclination to target is too large")
        if self.vessel.orbit.eccentricity > 0.1:
            raise Exception("Vessel eccentricity is too large")
        if self.target.orbit.eccentricity > 0.1:
            raise Exception("Target eccentricity is too large")
        
        self.connect_streams()

        transfer_angle = self.transfer_phase_angle()
        transfer_time = self.refine_transfer(transfer_angle)

        delta_v = self.transfer_delta_v(transfer_time)
        burn_time = self.burn_time(delta_v)
        node = self.vessel.control.add_node(transfer_time, prograde = delta_v)

        self.execute_next_node()
        # compute dv
        # setup node(s)
        # execute node(s)
