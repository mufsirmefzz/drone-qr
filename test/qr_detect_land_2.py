import cv2
import numpy as np
from pymavlink import mavutil

def connect_to_drone(connection_string='udp:127.0.0.1:14550'):
    print('Connecting to drone...')
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print('Drone connected')
    return master

def set_guided_mode(master):
    print("Setting mode to GUIDED...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )

def send_land_command(master):
    print('Sending land command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('Land command sent')

def main():
    rtsp_url = 'rtsp://192.168.144.25:8554/main.264'
    cap = cv2.VideoCapture(rtsp_url)
    
    if not cap.isOpened():
        print('Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()
    drone = connect_to_drone()
    set_guided_mode(drone)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('Failed to read frame')
            break

        # Convert to grayscale before QR detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        data, bbox, _ = qr_detector.detectAndDecode(gray_frame)

        if bbox is not None:
            bbox = bbox.astype(int)  # Ensure coordinates are integers
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        if data:
            print(f'QR Code detected: {data}')
            send_land_command(drone)
            break

        cv2.imshow('Drone Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()