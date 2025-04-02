import cv2
import numpy as np
import zxingcpp  # ZXing C++ wrapper for Python

# OpenCV to capture image
video_source = 'rtsp://192.168.144.25:8554/main.264'
cap = cv2.VideoCapture(video_source)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Decode using ZXing
    results = zxingcpp.read_barcodes(gray)

    for result in results:
        text = result.text
        position = result.position  

        if position:
            # Extract corner points
            pts = np.array([
                [position.top_left.x, position.top_left.y],
                [position.top_right.x, position.top_right.y],
                [position.bottom_right.x, position.bottom_right.y],
                [position.bottom_left.x, position.bottom_left.y]
            ], np.int32).reshape((-1, 1, 2))

            # Draw bounding box
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Display QR code text
        if text:
            cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            print(f'🔗 QR Code Detected: {text}')
    cv2.imshow("QR Code Scanner", frame)
    print
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
