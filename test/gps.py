COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 9  # Specified channel 11
SERVO_PWM_FORWARD = 2200  # Example PWM for forward motion
SERVO_PWM_CLOSE = 1100  # Example PWM for closing
SERVO_PWM_STOP = 1500

# Variables to store home location
home_latitude = None
home_longitude = None

# Set video source to UDP stream
video_source = 'udp://127.0.0.1:5600'
cap = cv2.VideoCapture(video_source)

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


def get_home_location(master):
    print('📍 Fetching home location from Flight Controller...')
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        global home_latitude, home_longitude
        home_latitude = msg.lat / 1e7
        home_longitude = msg.lon / 1e7
        print(f'✅ Home location saved: Latitude {home_latitude}, Longitude {home_longitude}')
    else:
        print('❌ Failed to fetch home location.')

def main:
    drone = connect_to_drone()
        if not drone:
            return

    get_home_location(drone)
        