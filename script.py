import krpc
import time
from common import notify
from launch import Launch
from circularize import Circularize
#from rendezvous import Rendezvous
from hohmann import HohmannTransfer
from landing import Land
from munar_return import ReturnToPlanet
from reentry import CapsuleReentry

conn = krpc.connect()
print(conn.krpc.get_status())

# Launch(conn, target_apoapsis=1e5, end_roll=5e4, target_heading=90.0).execute()
Launch(conn).execute()
Circularize(conn).execute()

HohmannTransfer(conn).execute()
Land(conn).execute()

# time.sleep(15)

# Launch(conn, target_apoapsis=5e4).execute()
# Circularize(conn).execute()
# ReturnToPlanet(conn).execute()
# CapsuleReentry(conn).execute()

notify("Done")