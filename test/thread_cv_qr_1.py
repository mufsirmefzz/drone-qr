import cv2
import threading
import numpy as np
from pymavlink import mavutil
import time

# MAVLink Connection
MAVLINK_CONNECTION = "COM8"  # Change this if needed
drone = mavutil.mavlink_connection(MAVLINK_CONNECTION)
drone.wait_heartbeat()
print("✅ Drone connected.")

class RTSPStream:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            print("❌ Failed to open RTSP stream.")
            self.running = False
        else:
            print("✅ RTSP Stream Opened Successfully.")

        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("⚠️ Frame read failed.")
                break

            with self.lock:
                self.frame = frame

        self.cap.release()

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.thread.join()

def detect_qr_code(frame):
    """Detects QR Code using OpenCV and computes its position."""
    detector = cv2.QRCodeDetector()
    data, bbox, _ = detector.detectAndDecode(frame)

    if bbox is None or data == "":
        return frame, None, None  # No QR detected

    bbox = np.int32(bbox[0])
    qr_x = int((bbox[0][0] + bbox[2][0]) / 2)
    qr_y = int((bbox[0][1] + bbox[2][1]) / 2)

    # Frame center
    frame_height, frame_width = frame.shape[:2]
    frame_center_x, frame_center_y = frame_width // 2, frame_height // 2

    # Compute Offsets (ΔX, ΔY)
    delta_x = qr_x - frame_center_x
    delta_y = qr_y - frame_center_y

    # Draw Bounding Box
    for i in range(len(bbox)):
        pt1 = tuple(bbox[i])
        pt2 = tuple(bbox[(i + 1) % len(bbox)])
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

    # Draw QR Code Center
    cv2.circle(frame, (qr_x, qr_y), 5, (0, 0, 255), -1)
    cv2.putText(frame, f"QR: {data}", (qr_x - 50, qr_y - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(frame, f"ΔX: {delta_x}, ΔY: {delta_y}", (qr_x - 50, qr_y + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    return frame, data, (delta_x, delta_y)

def move_towards_qr(delta_x, delta_y, threshold=30):
    """Moves drone based on QR code position offsets."""
    if abs(delta_x) < threshold and abs(delta_y) < threshold:
        print("🚀 Centered over QR. Initiating landing...")
        send_land_command()
        return

    roll = 1500  # Neutral roll
    pitch = 1500  # Neutral pitch

    # Adjust drone movement based on QR position
    if delta_x > threshold:
        roll = 1600  # Move right
    elif delta_x < -threshold:
        roll = 1400  # Move left

    if delta_y > threshold:
        pitch = 1600  # Move forward
    elif delta_y < -threshold:
        pitch = 1400  # Move backward

    send_rc_override(roll, pitch)

def send_rc_override(roll, pitch):
    """Sends RC override command to the drone."""
    print(f"📡 Sending RC Override: Roll={roll}, Pitch={pitch}")
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        roll, pitch, 1500, 1500, 0, 0, 0, 0
    )

def send_land_command():
    """Sends a LAND command to the drone."""
    print("🛬 Sending LAND command to drone.")
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def main():
    rtsp_url = "rtsp://192.168.144.25:8554/main.264"
    stream = RTSPStream(rtsp_url)

    while True:
        frame = stream.get_frame()
        if frame is not None:
            processed_frame, qr_data, deltas = detect_qr_code(frame)

            if processed_frame is not None:
                cv2.imshow("QR Code Tracking", processed_frame)

            if deltas is not None:
                print(f"QR Data: {qr_data}, Offset -> ΔX: {deltas[0]} px, ΔY: {deltas[1]} px")
                move_towards_qr(deltas[0], deltas[1])  # Move drone

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
