import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

detector = cv2.QRCodeDetector()

plt.ion()  # Turn on interactive mode for Matplotlib

while cap.isOpened():

    success, img = cap.read()
    if not success:
        print("Failed to capture image")
        break

    start = time.perf_counter()

    value, points, qrcode = detector.detectAndDecode(img)
    
    # Print out debug info
    if value != "":
        print("QR Code detected:", value)
    else:
        print("No QR Code detected")

    if value != "":

        # Draw bounding box and center circle for detected QR code
        x1 = points[0][0][0]
        y1 = points[0][0][1]
        x2 = points[0][2][0]
        y2 = points[0][2][1]

        x_center = (x2 - x1) / 2 + x1
        y_center = (y2 - y1) / 2 + y1

        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 5)
        cv2.circle(img, (int(x_center), int(y_center)), 3, (0, 0, 255), 3)

        # Display decoded QR code value with background for clarity
        cv2.putText(img, f'Decoded Value: {value}', (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        end = time.perf_counter()
        totalTime = end - start
        fps = 1 / totalTime

        # Display FPS
        cv2.putText(img, f'FPS: {int(fps)}', (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if value != "":
          print("FPS: ", fps)

    # Convert BGR to RGB (Matplotlib uses RGB)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Display the image using Matplotlib
    plt.imshow(img_rgb)
    plt.axis('off')  # Hide axes
    plt.draw()  # Update the plot
    plt.pause(0.0001)  # Pause to allow Matplotlib to update the window

    if cv2.waitKey(1) & 0xFF == 32:  # Press 'Esc' to exit
        break

cap.release()
cv2.destroyAllWindows()
