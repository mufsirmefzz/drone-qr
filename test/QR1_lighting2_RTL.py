import cv2
import numpy as np
import threading
import time
from pymavlink import mavutil

# MAVLink Configuration
COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 11  
SERVO_PWM_FORWARD = 2000  
SERVO_PWM_CLOSE = 1100  
RTL_ALTITUDE = 5  # Return altitude set to 5 meters

# RTSP Camera Configuration (Low Latency)
VIDEO_SOURCE = 'rtsp://192.168.144.25:8554/main.264'
cap = cv2.VideoCapture(VIDEO_SOURCE, cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# QR Code Detector
qr_detector = cv2.QRCodeDetector()

# Drone Connection
def connect_to_drone():
    """Establish connection to the drone."""
    print(f'🔗 Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat(timeout=2)
        print(f'✅ Connected! Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

# Set Servo Position
def set_servo(master, channel, pwm_value):
    """Move servo to specified PWM value."""
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        channel, pwm_value, 0, 0, 0, 0, 0
    )

# Wait for Landing
def wait_for_landing(master):
    """Wait until the drone lands."""
    print('⏳ Waiting for drone to land...')
    while True:
        msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if msg and msg.landed_state == 1:  # MAV_LANDED_STATE_ON_GROUND
            print('✅ Drone has landed.')
            break

# Get Home Location
def get_home_location(master):
    """Fetch home GPS location for return."""
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 24, 0, 0, 0, 0, 0, 0  # Request HOME_POSITION
    )
    time.sleep(1)
    
    while True:
        msg = master.recv_match(type="HOME_POSITION", blocking=True)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            print(f'🏠 Home location: {lat}, {lon}')
            return lat, lon
        time.sleep(1)

# Return to Home (Waypoints, 5m Altitude)
def return_to_home(master, home_lat, home_lon):
    """Return to home at 5m altitude using waypoints."""
    print(f'🚀 Returning to home at 5m altitude...')
    master.mav.mission_item_int_send(
        master.target_system, master.target_component, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0, int(home_lat * 1e7), int(home_lon * 1e7), RTL_ALTITUDE
    )
    print('✅ Waypoint command sent.')

# Land Drone
def land_drone(master, lat, lon):
    """Command drone to land at specific coordinates."""
    print('🛬 Landing drone...')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(lat * 1e7), int(lon * 1e7), 0,
        0, 0, 0, 0, 0, 0, 0, 0
    ))

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')

# Preprocess Image for Better QR Detection
def preprocess_image(frame):
    """Convert image to grayscale and apply adaptive thresholding."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY, 11, 2)
    return enhanced

# QR Code Detection Thread
def detect_qr(drone):
    """Detect QR codes and execute drone actions."""
    global cap
    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        processed_frame = preprocess_image(frame)
        data, bbox, _ = qr_detector.detectAndDecode(processed_frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 4)

            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

                threading.Thread(target=execute_mission, args=(drone,)).start()
                break

        cv2.imshow('QR Code Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Execute Drone Mission
def execute_mission(drone):
    """Land, operate servo, and return home."""
    home_lat, home_lon = get_home_location(drone)
    
    land_drone(drone, home_lat, home_lon)
    wait_for_landing(drone)

    print('⏳ Waiting 5 seconds after landing...')
    time.sleep(5)

    print('🔄 Rotating servo forward...')
    set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
    time.sleep(2)

    print('🔄 Rotating servo back...')
    set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)

    return_to_home(drone, home_lat, home_lon)

# Main Execution
def main():
    drone = connect_to_drone()
    if not drone:
        return

    detect_qr(drone)

    cap.release()
    cv2.destroyAllWindows()
    drone.close()
    print('🔌 Drone connection closed.')

if __name__ == '__main__':
    main()
