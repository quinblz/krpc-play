
from common import setup_staging_callback

class StagingAware():
    def __init__(self):
        self.conn = None
        self.vessel = None
        self.should_stage = False
        self.running = True

    def staging_callback(self, has_fuel):
        print(f"Staging callback: {has_fuel}")
        self.should_stage = not has_fuel

    def setup_staging_callback(self):
        return setup_staging_callback(self.conn, self.vessel, self.staging_callback)

    def check_staging(self, force=False):
        if self.should_stage or force:
            self.vessel.control.activate_next_stage()
            self.running = self.setup_staging_callback()
            return True
        return False