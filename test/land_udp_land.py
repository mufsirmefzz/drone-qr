import cv2
import socket
from pymavlink import mavutil

# Serial port settings
SERIAL_PORT = 'COM8'  # Change this to your drone's COM port
BAUD_RATE = 57600  # Match this with your Pixhawk settings

# UDP settings for QGC
UDP_IP = "127.0.0.1"
UDP_PORT = 14550

# Video source (0 = default webcam, change to external camera if needed)
VIDEO_SOURCE = 'rtsp://192.168.144.25:8554/main.264'  

def connect_to_drone():
    """ Connects to the drone via MAVLink on the specified COM port. """
    print(f'Connecting to drone on {SERIAL_PORT}...')
    try:
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print('✅ Drone connected on COM8')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def send_land_command(master):
    """ Sends a LAND command to the drone via MAVLink. """
    if master is not None:
        print('🛬 Sending LAND command...')
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print('✅ Land command sent to drone!')
    else:
        print('❌ No connection to drone!')

def forward_mavlink(master):
    """ Forwards MAVLink data from the drone to QGroundControl (QGC) over UDP. """
    print('📡 Forwarding MAVLink data to QGroundControl...')
    
    # Open UDP socket for sending data to QGC
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        msg = master.recv_msg()
        if msg:
            msg_bytes = msg.get_msgbuf()
            if msg_bytes:
                sock.sendto(msg_bytes, (UDP_IP, UDP_PORT))  # Send MAVLink data to QGC

def detect_qr_and_land(drone):
    """ Captures video, detects QR codes, and sends a LAND command if QR is found. """
    cap = cv2.VideoCapture(VIDEO_SOURCE)

    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue  # Keep trying to read frames

        # Detect QR code
        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)  # Draw bounding box

            if data:
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Show video feed
        cv2.imshow('QR Code Detection', frame)

        # If QR is detected, land the drone
        if data:
            print(f'✅ QR Code detected: {data}')
            send_land_command(drone)
            break  # Stop the loop after detecting the QR

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # Press 'q' to exit

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Connect to drone
    drone = connect_to_drone()
    
    if drone:
        # Run MAVLink forwarding in a separate thread
        import threading
        mav_thread = threading.Thread(target=forward_mavlink, args=(drone,))
        mav_thread.daemon = True
        mav_thread.start()

        # Start QR detection and land the drone when QR is detected
        detect_qr_and_land(drone)
