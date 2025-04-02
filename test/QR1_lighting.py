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
LATITUDE = 12.9716  
LONGITUDE = 77.5946  

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

def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')

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

def preprocess_frame(frame):
    """Preprocess frame for better QR code detection under high lighting conditions."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Adaptive thresholding to enhance contrast
    adaptive_thresh = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(adaptive_thresh, (5, 5), 0)

    # Use morphological transformations to refine QR code edges
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

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        # Preprocess the frame to improve QR code detection
        processed_frame = preprocess_frame(frame)

        # Detect and decode QR code
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

                print('🛬 Sending LAND command to given coordinates...')
                drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                    0, drone.target_system, drone.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
                ))

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

                print('🔄 Rotating servo 360 degrees forward...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(2)

                print('🔄 Rotating servo 360 degrees back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)

                return_to_launch(drone)
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
