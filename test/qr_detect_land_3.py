import cv2
import numpy as np
from pymavlink import mavutil

# Function to connect to the drone via COM7
def connect_to_drone(connection_string='COM8', baud_rate=57600):
    print(f'🔌 Connecting to drone on {connection_string}...')
    try:
        master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        master.wait_heartbeat()
        print('✅ Drone connected')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

# Function to set drone to GUIDED mode
def set_guided_mode(master):
    if master:
        print("🛩 Setting mode to GUIDED...")
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4  # GUIDED mode
        )

# Function to send LAND command to the drone
def send_land_command(master):
    if master:
        print("🛬 Sending LAND command...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

# Function to preprocess the frame for better QR detection
def preprocess_image(frame):
    """ Convert frame to grayscale and improve visibility """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Reduce noise
    return cv2.equalizeHist(gray)  # Increase contrast

def main():
    # UDP Video Stream Source
    udp_video_url = 'udp://192.168.144.25:8554'
    cap = cv2.VideoCapture(udp_video_url, cv2.CAP_FFMPEG)

    # Reduce delay
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()
    
    # Connect to the drone via COM7
    drone = connect_to_drone('COM7')

    if not drone:
        print("❌ Could not connect to the drone. Exiting...")
        return

    set_guided_mode(drone)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('⚠️ Failed to read frame')
            continue  # Keep trying to read frames

        processed_frame = preprocess_image(frame)
        data, bbox, _ = qr_detector.detectAndDecode(processed_frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)  # Draw bounding box

            # Display detected QR data
            if data:
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display video stream with bounding box
        cv2.imshow('Drone Camera - QR Detection', frame)

        # If QR is detected, land the drone
        if data:
            print(f'📌 QR Code detected: {data}')
            send_land_command(drone)
            break  # Stop the loop after detecting the QR

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to exit

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
