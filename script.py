import krpc
from common import notify
from launch import Launch
from circularize import Circularize
from rendezvous import Rendezvous

conn = krpc.connect()
print(conn.krpc.get_status())

vessel = conn.space_center.active_vessel

#Launch(conn).execute()
#Circularize(conn).execute()
Rendezvous(conn).execute()

notify("Done")