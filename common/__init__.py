import time
import math

def notify(*args):
    print(*args)

class Wait():
    def __init__(self,interval):
        self.interval = interval
    def __call__(self):
        time.sleep(self.interval)
wait = Wait(0.1)

def countdown_to(event_name):
    notify('3...')
    time.sleep(1)
    notify('2...')
    time.sleep(1)
    notify('1...')
    time.sleep(1)
    notify(event_name)

def clamp(x):
    return max(min(x, 1.0), 0.0)

def clamp_radians(x):
    tau = 2 * math.pi
    while x < 0:
        x += tau
    while x > tau:
        x -= tau
    return x

def highlight(part):
    part.highlight_color = (1.0, 0.0, 0.0)
    part.highlighted = True

surface_normal_gravity = 9.82
