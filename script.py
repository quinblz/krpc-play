import krpc
import time
import common
from launch import Launch
from circularize import Circularize

conn = krpc.connect()
print(conn.krpc.get_status())

vessel = conn.space_center.active_vessel

Launch(conn, vessel).execute()
Circularize(conn, vessel).execute()