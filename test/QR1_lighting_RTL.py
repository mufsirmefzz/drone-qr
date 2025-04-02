import cv2
import numpy as np
from pymavlink import mavutil
import time

# Set COM port, baud rate, and servo channel
COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 11  
SERVO_PWM_FORWARD = 2000  
SERVO_PWM_CLOSE = 1100  

def connect_to_drone():
    """Connect to the drone via MAVLink."""
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
    """Retrieve the home location (initial GPS coordinates)."""
    print('📍 Fetching home location...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    
    while True:
        msg = master.recv_match(type='HOME_POSITION', blocking=True)
        if msg:
            lat = msg.latitude / 1e7
            lon = msg.longitude / 1e7
            print(f'🏠 Home Location: Latitude={lat}, Longitude={lon}')
            return lat, lon

def set_servo(master, channel, pwm_value):
    """Move the servo to a specified PWM value."""
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')

def navigate_to_location(master, lat, lon, altitude=10):
    """Navigate the drone to a specified GPS location."""
    print(f'✈️ Navigating to GPS coordinates: {lat}, {lon}')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(lat * 1e7), int(lon * 1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0
    ))

def return_to_home(master, home_lat, home_lon):
    """Return to the home location at a 5m altitude."""
    print('🚀 Returning to home location at 5m altitude...')
    navigate_to_location(master, home_lat, home_lon, altitude=5)
    print('✅ Waypoint navigation to home initiated at 5m altitude.')

def preprocess_frame(frame):
    """Preprocess frame to enhance QR code detection."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    adaptive_thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    blurred = cv2.GaussianBlur(adaptive_thresh, (5, 5), 0)
    kernel = np.ones((3, 3), np.uint8)
    morph = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, kernel)
    return morph

def main():
    video_source = 'rtsp://192.168.144.25:8554/main.264'
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()
    drone = connect_to_drone()

    if not drone:
        print("❌ Unable to connect to drone. Exiting...")
        return

    # Fetch home location before proceeding
    home_lat, home_lon = get_home_location(drone)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        processed_frame = preprocess_frame(frame)
        data, bbox, _ = qr_detector.detectAndDecode(processed_frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Example coordinates from QR code (modify based on actual use case)
                detected_lat = 12.9716  
                detected_lon = 77.5946  

                # Navigate to QR detected location
                navigate_to_location(drone, detected_lat, detected_lon)

                print('⏳ Waiting for drone to reach location...')
                time.sleep(10)  # Wait for drone to reach (adjust based on speed)

                print('🔄 Rotating servo 360 degrees forward...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(2)

                print('🔄 Rotating servo 360 degrees back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)

                # Return to home location as waypoint
                return_to_home(drone, home_lat, home_lon)

                break

        cv2.imshow('QR Code Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if drone:
        drone.close()
        print('🔌 Drone connection closed.')

if __name__ == '__main__':
    main()
