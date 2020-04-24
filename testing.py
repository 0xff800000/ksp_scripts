import math
import time
import krpc
import pdb

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(
        name='My Example Program',
        address='192.168.1.104',
        rpc_port=50000, stream_port=50001)
print(conn.krpc.get_status().version)
vessel = conn.space_center.active_vessel

pdb.set_trace()
