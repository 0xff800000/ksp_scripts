import math
import time
import krpc
import pdb
import numpy as np

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(
        name='My Example Program',
        address='192.168.1.104',
        rpc_port=50000, stream_port=50001)
print(conn.krpc.get_status().version)
vessel = conn.space_center.active_vessel

# Set up streams for telemetry
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')

#stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)
# Get stages

root = vessel.parts.root
stack = [(root, 0)]
nb_stages = 8
stages_resources = [vessel.resources_in_decouple_stage(stage=i, cumulative=False) for i in range(0, nb_stages)]
stage_fuel = [conn.add_stream(st.amount, 'LiquidFuel') for st in stages_resources]

# Remove empty stages
def remove_empty_stages(stages):
    temp_stage = []
    for st in stages:
        if st() > 0:
            temp_stage.append(st)
    return temp_stage

def is_one_stage_empty(stages):
    ret = False
    for st in stages:
        if st() == 0:
            ret = True
    return ret

def circularize_at_apoapsis_delta_v():
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    # Compute v1 : velocity at apoapsis of current trajectory
    v1 = math.sqrt(mu*(2.0/r - 1/a1))
    # Compute v2 : velocity required to have a circular orbit
    v2 = math.sqrt(mu*(2.0/r - 1/a2))
    return v2-v1

def change_orbit_at_apoapsis_delta_v(radius):
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = radius
    # Compute v1 : velocity at apoapsis of current trajectory
    v1 = np.sqrt(mu*(2.0/radius - 1/a1))
    # Compute v2 : velocity required to have a circular orbit
    v2 = np.sqrt(mu*(2.0/radius - 1/a2))
    return v2-v1

def create_node_change_orbit_ap(altitude):
    print("Changing orbit from ap:{} to ap:{}".format(vessel.orbit.apoapsis_altitude,altitude))
    delta_v = change_orbit_at_apoapsis_delta_v(altitude+vessel.orbit.body.equatorial_radius)
    print("Delta V cost : {}".format(delta_v))
    burn_ut = ut() + vessel.orbit.time_to_apoapsis
    vessel.control.add_node(burn_ut, prograde = delta_v)
    
def hohmann_transfert(altitude):
    mu = vessel.orbit.body.gravitational_parameter
    r1 = vessel.orbit.apoapsis
    r2 = altitude + vessel.orbit.body.equatorial_radius
    delta_v1 = np.sqrt(mu/r1) * (np.sqrt(2*r2/(r1+r2)) - 1)
    delta_v2 = np.sqrt(mu/r2) * (1 - np.sqrt(2*r1/(r1+r2)))
    burn_ut1 = ut() + vessel.orbit.time_to_apoapsis
    vessel.control.add_node(burn_ut1, prograde = delta_v1)
    burn_ut2 = 0
    if delta_v2 > 0:
        burn_ut2 = burn_ut1 + vessel.control.nodes[0].orbit.time_to_apoapsis
    else:
        burn_ut2 = burn_ut1 + vessel.control.nodes[0].orbit.time_to_periapsis
    vessel.control.add_node(burn_ut2, prograde = delta_v2)


def get_magnitude(vect):
    return np.linalg.norm([vect[0], vect[1], vect[2]])

def get_burn_time(delta_v):
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    # Compute mass after burn
    mf = m0 / math.exp(delta_v / Isp)
    # Compute mass flow in kg/s
    kg_fuel_burn_p_s = F / Isp
    time = (m0 - mf) / kg_fuel_burn_p_s
    return time
        
def get_throttle(time,delta_v):
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    mf = m0 / math.exp(delta_v / Isp)
    kg_fuel_burn_p_s = (m0-mf) / time 
    F = kg_fuel_burn_p_s * Isp
    throttle = F / vessel.available_thrust
    return throttle

def create_node_circularize():
    dv = circularize_at_apoapsis_delta_v()
    burn_ut = ut() + vessel.orbit.time_to_apoapsis
    vessel.control.add_node(burn_ut, prograde = dv)
    
def execute_node(eps = 1.0):
    node = vessel.control.nodes[0]
    print("Executing node {} m/s planned in {}".format(node.delta_v,node.time_to))
    vessel.control.throttle = 0.0
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.engage()
    vessel.auto_pilot.wait()

    burn_time = get_burn_time(node.delta_v)

    print("Waiting until burn")
    burn_ut = node.ut - (burn_time / 2.0)
    leading_time = 10
    conn.space_center.warp_to(burn_ut - leading_time)
    
    time_to = conn.add_stream(getattr, vessel.control.nodes[0], 'time_to')
    while time_to() - (burn_time/2.0) > 0:
        pass

    print("Start burn")
    original_burn_dv = node.remaining_delta_v
    if burn_time > 5:
        vessel.control.throttle = 1.0
        time.sleep(burn_time * 0.95)
    else:
        # Burn for at least some time
        new_burn_time = 2
        vessel.control.throttle = get_throttle(new_burn_time, node.remaining_delta_v)
        while node.remaining_delta_v > original_burn_dv*0.1:
            pass


    print("Fine tune 1")
    vessel.control.throttle = 0.05
    #remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    #while get_magnitude(remaining_burn()) > eps:
    while node.remaining_delta_v > eps:
        pass
    print("Fine tune 2")
    vessel.control.throttle = 0.025
    #while get_magnitude(remaining_burn()) > eps/2:
    while node.remaining_delta_v > eps:
        pass
    vessel.control.throttle = 0.0
    print("Burn done")
    node.remove()
    print("Node executed")


def print_stages():
    for i,st in enumerate(stage_fuel):
        print("Stage {}: {}".format(i,st()))

def ascent_from_kerbin():
    # Pre-launch setup
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0
    stage_fuel = remove_empty_stages(stage_fuel)
    print_stages()
    current_stage = nb_stages

    # Countdown...
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')

        
    # Activate the first stage
    vessel.control.activate_next_stage()
    current_stage -= 1
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # Main ascent loop
    turn_angle = 0
    while True:

        # Gravity turn
        if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # Separate SRBs when finished
        if current_stage != 1:
            if is_one_stage_empty(stage_fuel):
                vessel.control.activate_next_stage()
                print('Stage {} separated'.format(current_stage))
                stage_fuel = remove_empty_stages(stage_fuel)
                current_stage -= 1
                print_stages()

        # Decrease throttle when approaching target apoapsis
        if apoapsis() > target_altitude*0.9:
            print('Approaching target apoapsis')
            break

    # Burn until apoapsis is at target
    vessel.control.throttle = 0.25
    while apoapsis() > target_altitude:
        pass
    vessel.control.throttle = 0.0


    create_node_circularize()
    execute_node()

pdb.set_trace()
