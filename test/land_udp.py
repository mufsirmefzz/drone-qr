import socket
from pymavlink import mavutil

# Serial port settings
SERIAL_PORT = 'COM8'  # Change to your drone's COM port
BAUD_RATE = 57600  # Match this with your Pixhawk settings

# UDP settings for QGC
UDP_IP = "127.0.0.1"
UDP_PORT = 14550

def forward_mavlink():
    """ Reads MAVLink data from COM8 and forwards it to QGroundControl via UDP. """
    print(f'Connecting to drone on {SERIAL_PORT}...')
    
    try:
        # Establish connection with the drone
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print('✅ Drone connected on COM8')

        # Open UDP socket for sending data to QGC
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while True:
            # Read MAVLink messages from serial
            msg = master.recv_msg()
            if msg:
                msg_bytes = msg.get_msgbuf()
                if msg_bytes:
                    sock.sendto(msg_bytes, (UDP_IP, UDP_PORT))  # Send MAVLink data to QGC
    except Exception as e:
        print(f'❌ Error: {e}')

if __name__ == "__main__":
    forward_mavlink()
