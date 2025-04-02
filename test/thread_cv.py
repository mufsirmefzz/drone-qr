import cv2
import threading

class RTSPStream:
    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        # OpenCV VideoCapture (runs in a separate thread)
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)

        if not self.cap.isOpened():
            print("❌ Failed to open RTSP stream.")
            self.running = False
        else:
            print("✅ RTSP Stream Opened Successfully.")

        # Start the thread
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def update(self):
        """Continuously grabs the latest frame."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("⚠️ Failed to grab frame.")
                break

            with self.lock:
                self.frame = frame  # Store only the latest frame

        self.cap.release()

    def get_frame(self):
        """Retrieve the latest frame safely."""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """Stops the thread and releases resources."""
        self.running = False
        self.thread.join()

def main():
    rtsp_url = "rtsp://192.168.144.25:8554/main.264"
    stream = RTSPStream(rtsp_url)

    while True:
        frame = stream.get_frame()
        if frame is not None:
            cv2.imshow("RTSP Stream (Threaded)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
