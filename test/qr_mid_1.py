import cv2
import time
import numpy as np
from pymavlink import mavutil

COM_PORT = 'COM8'
BAUD_RATE = 57600  # Define baud rate
VIDEO_SOURCE = 'rtsp://192.168.144.25:8554/main.264'
THRESHOLD = 10  # Pixel tolerance for alignment
MOVE_STEP = 0.1  # Movement step in meters
ALPHA = 0.1  # Pixels per cm (calibrate for your camera and altitude)
OFFSET_Y_PIXELS = int(5 / ALPHA)  # Convert 5 cm camera offset to pixels

def connect_to_drone():
    """Establish connection with the drone."""
    print(f'Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print(f'✅ Drone connected. Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def detect_qr_code(frame, detector):
    """Detects QR Code and returns its center coordinates only if valid."""
    data, bbox, _ = detector.detectAndDecode(frame)

    if bbox is not None and len(bbox) > 0 and data:
        bbox = bbox[0]  # Extract bounding box points
        bbox = np.int32(bbox)  # Convert all points to integers

        # Compute QR Code Midpoint
        qr_x = int((bbox[0][0] + bbox[2][0]) / 2)
        qr_y = int((bbox[0][1] + bbox[2][1]) / 2)

        # Draw Bounding Box and Center
        for i in range(len(bbox)):
            pt1 = tuple(bbox[i])
            pt2 = tuple(bbox[(i+1) % len(bbox)])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        cv2.circle(frame, (qr_x, qr_y), 5, (0, 0, 255), -1)

        return qr_x, qr_y

    return None, None

def send_position_ned(master, x, y, z):
    """Sends position setpoint in Local NED coordinates."""
    master.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,  # Only position enabled
        x, y, z,
        0, 0, 0,  # No velocity
        0, 0, 0,  # No acceleration
        0, 0  # No yaw change
    )

def move_drone_to_qr(master, error_x, error_y):
    """Prints the direction in which the drone should move."""
    if abs(error_x) > THRESHOLD:
        if error_x > 0:
            print("Move Right")
        else:
            print("Move Left")

    if abs(error_y) > THRESHOLD:
        if error_y > 0:
            print("Move Forward")
        else:
            print("Move Backward")

    return abs(error_x) < THRESHOLD and abs(error_y) < THRESHOLD  # Stop when errors are small

def land(master):
    """Commands the drone to land."""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0
    )
    print("🛬 Landing...")

def main():
    """Main function to handle video processing and drone movement."""
    cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return 

    detector = cv2.QRCodeDetector()
    master = connect_to_drone()
    if master is None:
        cap.release()
        return

    qr_stable_counter = 0  # Counter to check stable detection

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Detect QR Code
        qr_x, qr_y = detect_qr_code(frame, detector)

        if qr_x is not None and qr_y is not None:
            qr_y_corrected = qr_y - OFFSET_Y_PIXELS  # Apply Camera Offset Correction

            # Get Drone's Viewport Center
            frame_height, frame_width, _ = frame.shape
            drone_x, drone_y = frame_width // 2, frame_height // 2

            # Calculate Position Errors
            error_x = qr_x - drone_x
            error_y = qr_y_corrected - drone_y

            # Move Drone Toward QR Code
            aligned = move_drone_to_qr(master, error_x, error_y)

            # Ensure QR is stable for 5 frames before landing
            if aligned:
                qr_stable_counter += 1
            else:
                qr_stable_counter = 0  # Reset if unstable

            if qr_stable_counter >= 5:
                print("✅ Stable QR detected. Ready to land.")
                land(master)
                break  # Exit loop after landing

        # Display Frame
        cv2.imshow("QR Code Landing", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
