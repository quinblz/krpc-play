import time

def wait():
    time.sleep(0.1)

def countdown_to(event_name):
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print(event_name)

def clamp(x):
    return max(min(x, 1.0), 0.0)

def setup_staging_callback(conn, vessel, callback):
    stage_stream = get_fuel_connection(conn, vessel)
    if stage_stream is None:
        callback(False)
        return False
    stage_stream.add_callback(callback)
    stage_stream.start()
    return True

def get_fuel_connection(conn, vessel):
    burnout = stageable_engine(vessel)
    return burnout and conn.add_stream(getattr, burnout, 'has_fuel')

def stageable_engine(vessel):
    active_engines = [e for e in vessel.parts.engines if e.active]
    if not active_engines:
        return None
    def priority(engine):
        return -engine.part.decouple_stage
    burnout = sorted(active_engines, key=priority)[0]
    highlight(burnout.part)
    return burnout

def highlight(part):
    part.highlight_color = (1.0, 0.0, 0.0)
    part.highlighted = True
