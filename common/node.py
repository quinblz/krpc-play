######################################################################
### Node Execution Library and Example
######################################################################
###   Like all of the scripts in my folder here, this file contains
###   functions you might want to include into your own scripts for  
###   actual use and a demo in the 'main' function that you can just 
###   run to see how it works.
###
###  This file shows how to execute maneuver nodes.    The docstring
###  for the execute_next_node function explains how to make this work!
###  And you can see an example of doing so in the launch script.
###  Credit: https://github.com/krpc/krpc-library/blob/master/Art_Whaleys_KRPC_Demos/node_executor.py
######################################################################

import krpc
import math
import time
from common import clamp, wait

def main():
    conn = krpc.connect()
#Demo of all three major functions in this file - uncomment the one you want!
    execute_btn(conn)   #Creates an on screen button to execute the next node
  #  execute_next_node(conn)  #Executes the next node!
  #  execute_all_nodes(conn)       #executes ALL nodes instead of just the next one!
 
def execute_next_node(conn):
    '''
    This is the actually interesting function in this script!
    Executes the Next Maneuver Node for the vessel provided.
    If you just open and run this file, it will execute a node and exit.
    You can also include this file into your own script with the line
    from node_executor import execute_next_node
    at the top of your script, and then anytime you want to execute a node
    you just have to call (execute_next_node(conn) passing it the active 
    KRPC connection as a parameter.
    I'm also demonstrating two different ways to point the vessel with the
    autopilot.  One relies on the vessel having SAS Node holding capabilty,
    the other uses the KRPC built-in auto-pilot.   The one built into
    KRPC can require some tuning depending on your vessel...  but works on
    any vessel regardless of pilot skill/probe core choice!   
    '''
    space_center = conn.space_center
    vessel = space_center.active_vessel
    ap=vessel.auto_pilot

# Grab the next node if it exists
    try:
        node = vessel.control.nodes[0]
    except Exception:
        return    #Fail silently but gracefully if there was no node to execute
    
    
# Orient vessel to the node
################## One Way To Orient Vessel!##############
    rf = vessel.orbit.body.reference_frame
    ap.reference_frame=rf
    ap.engage()
    ap.target_direction = node.remaining_burn_vector(rf)
    ap.wait()

##################  Another Way To Orient Vessel!########
    #ap.sas = True
    #time.sleep(.1)
    #ap.sas_mode = vessel.auto_pilot.sas_mode.maneuver
    #ap.wait()
        
# Calculate the length and start of burn
    m = vessel.mass
    isp = vessel.specific_impulse
    dv = node.delta_v
    F = vessel.available_thrust
    G = 9.81
    burn_time = (m - (m / math.exp(dv / (isp * G)))) / (F / (isp * G))

# TODO: check fuel level for potential staging

# Warp until burn
    space_center.warp_to(node.ut - (burn_time / 2.0) - 5.0)
    while node.time_to > (burn_time / 2.0):
        pass
    ap.wait()
    
# Actually Burn
    vessel.control.throttle, acc = thrust_controller(vessel, node.remaining_delta_v)  
    while node.remaining_delta_v > acc * wait.interval:
        ap.target_direction=node.remaining_burn_vector(rf)
        vessel.control.throttle, acc = thrust_controller(vessel, node.remaining_delta_v)
        wait()

# Finish Up
    ap.disengage()
    vessel.control.throttle = 0.0
    node.remove()

def execute_all_nodes(conn):

    '''
    as the name implies - this function executes ALL maneuver nodes currently
    planned for the vessel in series.
    '''
    space_center = conn.space_center
    vessel = space_center.active_vessel
    while vessel.control.nodes:
        execute_next_node(conn)

def thrust_controller(vessel, deltaV):
    '''
    This function is somewhat arbitrary in it's working - there's not a 'rule'
    that I've found on how to feather out throttle towards the end of a burn
    but given that the chances of overshooting go up with TWR (since we fly
    in a world with discrete physics frames!) it makes sense to relate the
    throttle to the TWR for this purpose.
    '''
    if not vessel.available_thrust:
        vessel.control.activate_next_stage()
        time.sleep(0.1)
    TWR= vessel.available_thrust/vessel.mass
    min_throttle = 0.05
    throttle = min_throttle + deltaV / TWR / 5
    throttle = clamp(throttle)
    return throttle, TWR * throttle