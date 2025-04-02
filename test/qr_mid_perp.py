import cv2
import numpy as np
from pymavlink import mavutil
import time

# MAVLink connection
connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Camera parameters (calibrate for your setup)
camera_matrix = np.array([[900, 0, 320], [0, 900, 240], [0, 0, 1]])
dist_coeffs = np.zeros((4, 1))
TARGET_SIZE = 0.5  # Physical QR code size in meters

def send_landing_target(distance, x_offset, y_offset):
    msg = connection.mav.landing_target_encode(
        time.time_ns() // 1000,  # Timestamp (microseconds)
        0,                       # Target num
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        x_offset,                # Angular X offset
        y_offset,                # Angular Y offset
        distance,                # Distance to target
        0, 0                     # Size (unused)
    )
    connection.mav.send(msg)

def qr_landing_controller():
    cap = cv2.VideoCapture(0)
    qr_detector = cv2.QRCodeDetector()
    landing_threshold = 0.1  # 10cm accuracy
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # QR code detection
        ret_qr, points = qr_detector.detect(frame)
        if ret_qr:
            # Calculate position using Perspective-n-Point
            obj_pts = np.array([
                [-TARGET_SIZE/2, -TARGET_SIZE/2, 0],
                [TARGET_SIZE/2, -TARGET_SIZE/2, 0],
                [TARGET_SIZE/2, TARGET_SIZE/2, 0],
                [-TARGET_SIZE/2, TARGET_SIZE/2, 0]
            ], dtype=np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_pts,
                points.astype(np.float32),
                camera_matrix,
                dist_coeffs
            )

            if success:
                # Convert translation vector to distance/offsets
                distance = np.linalg.norm(tvec)
                x_offset = tvec[0][0]
                y_offset = tvec[1][0]

                # Send landing target data
                send_landing_target(distance, x_offset, y_offset)

                # Exit condition
                if distance < landing_threshold:
                    connection.mav.command_long_send(
                        connection.target_system,
                        connection.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                    break

        time.sleep(0.1)

    cap.release()

if __name__ == "__main__":
    # Wait for heartbeat
    connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % 
          (connection.target_system, connection.target_component))
    
    # Start landing procedure
    qr_landing_controller()
