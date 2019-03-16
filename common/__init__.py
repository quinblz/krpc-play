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

def highlight(part):
    part.highlight_color = (1.0, 0.0, 0.0)
    part.highlighted = True
