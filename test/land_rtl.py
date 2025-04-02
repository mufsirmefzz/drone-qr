import numpy as np
from pymavlink import mavutil
import time

COM_PORT = 'COM8'  
BAUD_RATE = 57600

def connect_to_drone():
    print(f'Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print(f'✅ Drone connected. Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def wait_for_landing(master):
    print('⏳ Waiting for drone to land...')
    while True:
        msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if msg and msg.landed_state == 1:  # MAV_LANDED_STATE_ON_GROUND
            print('✅ Drone has landed.')
            break    

def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')

def main():
    drone = connect_to_drone()
    while True:
        print('🛬 Sending LAND command...')
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print('✅ Land command sent.')

        wait_for_landing(drone)

        print('⏳ Waiting for 5 seconds after landing...')
        time.sleep(5)

        return_to_launch(drone)
        break
