import cv2
import numpy as np
from pymavlink import mavutil
import time

# Set COM port, baud rate, and servo channel
COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 1  # Set to channel 1
SERVO_PWM_FORWARD = 20000  # Example PWM for forward motion
SERVO_PWM_CLOSE = 1100  # Example PWM for closing


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
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')


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

        data, bbox, _ = qr_detector.detectAndDecode(frame)

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

                print('⏳ Waiting for 5 seconds...')
                time.sleep(100)

                print('🔄 Rotating servo 360 degrees forward...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(2)

                print('🔄 Rotating servo 360 degrees back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
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
