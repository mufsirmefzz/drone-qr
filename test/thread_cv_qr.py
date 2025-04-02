import cv2
import threading
import numpy as np
from pyzbar.pyzbar import decode  # More reliable QR detection

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
        """Continuously grabs the latest frame from RTSP."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("⚠️ Frame read failed.")
                break

            with self.lock:
                self.frame = frame

        self.cap.release()

    def get_frame(self):
        """Retrieve the latest frame safely."""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """Stops the thread and releases resources."""
        self.running = False
        self.thread.join()

def detect_qr_code(frame):
    """
    Detects QR Code and computes X, Y distance from the center.
    Uses OpenCV + Pyzbar for better accuracy.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to Grayscale
    detector = cv2.QRCodeDetector()
    
    # First try OpenCV's QR detection
    data, bbox, _ = detector.detectAndDecode(gray)

    if bbox is not None and data:
        bbox = np.int32(bbox[0])
        qr_x = int((bbox[0][0] + bbox[2][0]) / 2)
        qr_y = int((bbox[0][1] + bbox[2][1]) / 2)
    else:
        # Fallback to Pyzbar if OpenCV fails
        decoded_objects = decode(gray)
        if len(decoded_objects) == 0:
            return frame, None, None  # No QR detected

        for obj in decoded_objects:
            data = obj.data.decode("utf-8")
            bbox = obj.polygon  # Get the bounding box
            if len(bbox) == 4:  # Ensure it has 4 corners
                bbox = np.array([[p.x, p.y] for p in bbox], dtype=np.int32)
                qr_x = int((bbox[0][0] + bbox[2][0]) / 2)
                qr_y = int((bbox[0][1] + bbox[2][1]) / 2)
                break  # Stop at first detected QR

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
                print(f"QR Code Data: {qr_data}, Offset -> ΔX: {deltas[0]} pixels, ΔY: {deltas[1]} pixels")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
