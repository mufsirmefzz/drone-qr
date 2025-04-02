from pymavlink import mavutil
import time

COM_PORT = 'COM8' # Example for serial connection
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

def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, # Confirmation
        0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')

# Example usage
if __name__ == "__main__":
    master = connect_to_drone()
    time.sleep(30)
    if master:
        return_to_launch(master)
