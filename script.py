import krpc
import time
conn = krpc.connect()
print(conn.krpc.get_status())

ksc = conn.space_center
vessel =ksc.active_vessel
ap = vessel.auto_pilot

vessel.control.throttle = 1.0
ap.target_pitch_and_heading(90, 0)
ap.engage()
time.sleep(0.1)

if(vessel.situation == ksc.VesselSituation.pre_launch):
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print("LAUNCH!")
    vessel.control.activate_next_stage()

while True:
    active_engines = [e for e in vessel.parts.engines if e.active]
    if not active_engines:
        break
    def get_stage(engine):
        return -engine.part.decouple_stage
    burnout = sorted(active_engines, key=get_stage)[0]
    burnout.part.highlight_color = (1.0, 0.0, 0.0)
    burnout.part.highlighted = True
    with conn.stream(getattr, burnout, 'has_fuel') as has_fuel:
        with has_fuel.condition:
            while has_fuel():
                has_fuel.wait()
        vessel.control.activate_next_stage()
print("Done.")
