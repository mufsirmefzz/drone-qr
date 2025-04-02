import cv2
import numpy as np
from pymavlink import mavutil

# Set the serial port and baud rate
COM_PORT = 'COM8'  # Change this to your actual COM port
BAUD_RATE = 57600  # Match with your Pixhawk settings

def connect_to_drone():
    print(f'Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print('✅ Drone connected')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def send_land_command(master):
    if master is not None:
        print('🛬 Sending land command...')
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print('✅ Land command sent')
    else:
        print('❌ No connection to drone!')

# Connect and send LAND command
drone = connect_to_drone()
if drone:
    send_land_command(drone)
