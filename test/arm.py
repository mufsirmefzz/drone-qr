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

def arm_drone(master):
    print('🔋 Arming the drone...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to arm
        0, 0, 0, 0, 0, 0
    )
    print('✅ Drone armed.')

def disarm_drone(master):
    print('⏳ Waiting for 2 seconds before disarming...')
    time.sleep(5)
    print('🔋 Disarming the drone...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0 to disarm
        0, 0, 0, 0, 0, 0
    )
    print('✅ Drone disarmed.')

def main():
    drone = connect_to_drone()
    if drone:
        time.sleep(10)
        arm_drone(drone)
        disarm_drone(drone)

if __name__ == '__main__':
    main()
