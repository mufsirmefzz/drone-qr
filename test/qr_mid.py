import cv2
import time
import numpy as np
from pymavlink import mavutil

# MAVLink Connection (USB or UDP)
drone = mavutil.mavlink_connection('COM7')  # Use correct port for hardware
drone.wait_heartbeat()
print("✅ Drone connected!")

# Initialize Camera
cap = cv2.VideoCapture(0)  # Adjust if using an external camera
detector = cv2.QRCodeDetector()

# Define Parameters
THRESHOLD = 10  # Pixel tolerance for alignment
MOVE_STEP = 0.1  # Movement step in meters
ALPHA = 0.1  # Pixels per cm (calibrate for your camera and altitude)
OFFSET_Y_PIXELS = int(5 / ALPHA)  # Convert 5 cm camera offset to pixels

def detect_qr_code(frame):
    """Detects QR Code and returns its center coordinates."""
    data, bbox, _ = detector.detectAndDecode(frame)

    if bbox is not None:
        bbox = bbox[0]  # Extract bounding box points
        
        # Compute QR Code Midpoint
        qr_x = int((bbox[0][0] + bbox[2][0]) / 2)
        qr_y = int((bbox[0][1] + bbox[2][1]) / 2)

        # Draw Bounding Box and Center
        for i in range(len(bbox)):
            cv2.line(frame, tuple(bbox[i]), tuple(bbox[(i+1) % len(bbox)]), (0, 255, 0), 2)
        cv2.circle(frame, (qr_x, qr_y), 5, (0, 0, 255), -1)

        return qr_x, qr_y
    return None, None

def send_position_ned(drone, x, y, z):
    """Sends position setpoint in Local NED coordinates."""
    drone.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,  # Only position enabled
        x, y, z,
        0, 0, 0,  # No velocity
        0, 0, 0,  # No acceleration
        0, 0  # No yaw change
    )

def move_drone_to_qr(drone, error_x, error_y):
    """Moves drone towards the QR Code midpoint."""
    if abs(error_x) > THRESHOLD:
        if error_x > 0:
            send_position_ned(drone, MOVE_STEP, 0, 0)  # Move Right
        else:
            send_position_ned(drone, -MOVE_STEP, 0, 0)  # Move Left

    if abs(error_y) > THRESHOLD:
        if error_y > 0:
            send_position_ned(drone, 0, MOVE_STEP, 0)  # Move Forward
        else:
            send_position_ned(drone, 0, -MOVE_STEP, 0)  # Move Backward

    return abs(error_x) < THRESHOLD and abs(error_y) < THRESHOLD  # Stop when errors are small

def land(drone):
    """Commands the drone to land."""
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0
    )
    print("🛬 Landing...")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Detect QR Code
    qr_x, qr_y = detect_qr_code(frame)

    if qr_x is not None and qr_y is not None:
        # Apply Camera Offset Correction
        qr_y_corrected = qr_y - OFFSET_Y_PIXELS

        # Get Drone's Viewport Center
        frame_height, frame_width, _ = frame.shape
        drone_x, drone_y = frame_width // 2, frame_height // 2

        # Calculate Position Errors
        error_x = qr_x - drone_x
        error_y = qr_y_corrected - drone_y

        # Move Drone Toward QR Code
        aligned = move_drone_to_qr(drone, error_x, error_y)

        # If Aligned, Land the Drone
        if aligned:
            land(drone)
            break

    # Display Frame
    cv2.imshow("QR Code Landing", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
