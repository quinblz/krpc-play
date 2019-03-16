import krpc
import time
import common

conn = krpc.connect()
print(conn.krpc.get_status())

ksc = conn.space_center
vessel =ksc.active_vessel
ap = vessel.auto_pilot

running = True
should_stage = False
target_heading = 90.0
target_apoapsis = 150000

altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
mass = conn.add_stream(getattr, vessel, 'mass')
available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')

def wait():
    time.sleep(0.1)

def clamp(x):
    return max(min(x, 1.0), 0.0)

def staging_callback(has_fuel):
    global should_stage
    print(f"Staging callback: {has_fuel}")
    should_stage = not has_fuel

def check_staging(force=False):
    global running
    if should_stage or force:
        vessel.control.activate_next_stage()
        running = common.setup_staging_callback(conn, vessel, staging_callback)
        return True
    return False

def g_limit(g):
    target_throttle = 0.0
    available_accl = available_thrust() / mass()
    target_accl = g * 9.8
    if available_accl:
        target_throttle = clamp(target_accl / available_accl)
    vessel.control.throttle = target_throttle

def gravity_turn():
    start_roll = 250
    end_roll = 45000
    completion = clamp((altitude() - start_roll) / (end_roll - start_roll))
    target_pitch = 90 * (1.0 - completion)
    if abs(target_pitch - ap.target_pitch) > 0.5:
        ap.target_pitch_and_heading(target_pitch, target_heading)
    
vessel.control.throttle = 1.0
ap.engage()

if(vessel.situation == ksc.VesselSituation.pre_launch):
    ap.target_pitch_and_heading(90, target_heading)
    common.countdown()
    check_staging(force=True)
else:
    common.setup_staging_callback(conn, vessel, staging_callback)

while running:
    staged = check_staging()

    if apoapsis() < 0.9 * target_apoapsis:
        g_limit(2.2)
        gravity_turn()
    elif apoapsis() < target_apoapsis:
        g_limit(1.0)
    else:
        vessel.control.throttle = 0.0
        wait()
        break
    wait()
print("Done.")
