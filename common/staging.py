from common import highlight

class StagingAware():
    def __init__(self):
        self.conn = None
        self.vessel = None
        self.should_stage = False
        self.running = True

        self.__fuel_monitor = None

    def setup_staging_callback(self):
        self.update_fuel_monitor()
        if self.__fuel_monitor is None:
            self.staging_callback(False)
            return False
        self.__fuel_monitor.add_callback(self.staging_callback)
        self.__fuel_monitor.start()
        return True

    def update_fuel_monitor(self):
        engine = self.get_stageable_engine()
        if not engine:
            self.__fuel_monitor = None
            return
        highlight(engine.part)
        self.__fuel_monitor = self.conn.add_stream(getattr, engine, 'has_fuel')

    def get_stageable_engine(self):
        active_engines = [e for e in self.vessel.parts.engines if e.active]
        if not active_engines:
            return None
        def priority(engine):
            return -engine.part.decouple_stage
        burnout = sorted(active_engines, key=priority)[0]
        return burnout

    def staging_callback(self, has_fuel):
        self.should_stage = not has_fuel

    def check_staging(self, force=False):
        if self.should_stage or force:
            self.vessel.control.activate_next_stage()
            self.running = self.setup_staging_callback()
            return True
        return False