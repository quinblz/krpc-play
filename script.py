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

vessel = conn.space_center.active_vessel

Launch(conn).execute()
exit()
Circularize(conn).execute()

#vessel.control.activate_next_stage()

HohmannTransfer(conn).execute()
Land(conn).execute()

time.sleep(15)

Launch(conn, target_apoapsis=5e4).execute()
Circularize(conn).execute()
ReturnToPlanet(conn).execute()
CapsuleReentry(conn).execute()

notify("Done")