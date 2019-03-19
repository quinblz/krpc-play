

class Rendezvous():
    def __init__(self, conn, vessel, target):
        self.conn = conn
        self.vessel = vessel
        self.target = target

    def execute(self):
        if self.vessel.orbit.eccentricity > 0.1:
            raise Exception("Eccentricity is too large")
        if self.vessel.orbit.relative_inclination(self.target.orbit) > 0.005:
            raise Exception("Inclination to target is too large")
        
        # half hohman transfer to apoapsis/periapsis
        # setup encounter
        # match orbit
        # close approach