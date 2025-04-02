import cv2
import numpy as np
from pymavlink import mavutil
import time

# Set COM port, baud rate, and servo channel
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
cap = cv2.VideoCapture(video_source, cv2.CAP_FFMPEG)

if not cap.isOpened():
    print('❌ Error: Unable to open video stream.')
    exit()

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
    for _ in range(10):
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            global home_latitude, home_longitude
            home_latitude = msg.lat / 1e7
            home_longitude = msg.lon / 1e7
            print(f'✅ Home location saved: Latitude {home_latitude}, Longitude {home_longitude}')
            return
        print('⚠️ Retrying to fetch home location...')
    print('❌ Failed to fetch home location.')

def set_guided_mode(master):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system, 0,
        mavutil.mavlink.MAV_MODE_GUIDED_ARMED
    )
    print('✅ Mode set to GUIDED.')

def arm_and_takeoff(master, altitude=5):
    print('🕹️ Arming drone...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Drone armed.')

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(10)

def go_to_home(master, altitude=5):
    print('🧭 Navigating to home location...')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(home_latitude * 1e7), int(home_longitude * 1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0
    ))
    print('✅ Heading to home.')

def set_servo(master, channel, pwm_value):
    print(f'🔧 Setting servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    print('✅ Servo command sent.')

def land_drone(master):
    print('🛬 Initiating landing...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')

def main():
    drone = connect_to_drone()
    if not drone:
        return

    get_home_location(drone)

    print('🔧 Setting servo to 1500 PWM...')
    set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)
    time.sleep(2)

    qr_detector = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:
                print(f'✅ QR Code detected: {data}')
                set_guided_mode(drone)
                land_drone(drone)
                print('⏳ Waiting for landing completion...')
                time.sleep(10)

                print('🔄 Rotating servo forward for 3s...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(3)

                print('🔄 Rotating servo backward for 3s...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
                time.sleep(3)

                print('🛑 Stopping servo...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)

                arm_and_takeoff(drone, altitude=5)
                go_to_home(drone, altitude=5)
                land_drone(drone)
                break

        cv2.imshow('QR Code Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    print('✅ Mission completed.')

if __name__ == '__main__':
    main()
