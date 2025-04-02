import cv2
import numpy as np
from pymavlink import mavutil

# Set the serial port and baud rate
COM_PORT = 'COM8'  # Change this to your actual COM port
BAUD_RATE = 57600  # Match with your Pixhawk settings

def connect_to_drone():
    """ Connects to the drone via MAVLink on the specified COM port. """
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
    """ Sends a LAND command to the drone via MAVLink. """
    if master is not None:
        print('🛬 Sending LAND command...')
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print('✅ Land command sent')
    else:
        print('❌ No connection to drone!')

def main():
    """ Captures video, detects QR codes, and sends a LAND command if QR is found. """
    video_source = 'rtsp://192.168.144.25:8554/main.264'  # Change this if using an external camera
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
            continue  # Keep trying to read frames

        # Detect and decode QR code
        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)  # Draw bounding box

            # Display detected QR data
            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                send_land_command(drone)  # Send LAND command on QR detection
                break  # Stop after detecting QR code

        # Show video feed
        cv2.imshow('QR Code Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to exit

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
