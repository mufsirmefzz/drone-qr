import av
import cv2
import numpy as np

RTSP_URL = "rtsp://192.168.144.25:8554/main.264"

container = av.open(RTSP_URL)
for frame in container.decode(video=0):
    img = frame.to_ndarray(format="bgr24")  # Convert frame to OpenCV format
    cv2.imshow("RTSP Stream", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
