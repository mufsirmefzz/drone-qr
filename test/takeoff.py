# Import necessary libraries
import cv2
import numpy as np
from pymavlink import mavutil
import time

# Configuration Variables
COM_PORT = 'COM8'
BAUD_RATE = 57600
SERVO_CHANNEL = 9
SERVO_PWM_FORWARD = 2200
SERVO_PWM_CLOSE = 1100
SERVO_PWM_STOP = 1500
LATITUDE = 10.0524451
LONGITUDE = 76.620354
VIDEO_SOURCE = 'rtsp://192.168.144.25:8554/main.264'

# Connect to Drone
def connect_to_drone():
    print(f'Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print(f'✅ Drone connected. Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

# Control Servo
def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')

# Drone Mode Control
def set_guided_mode(master):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 represents GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

# Arm and Takeoff
'''def arm_and_takeoff(master, altitude=5):
    print('🕹️ Arming drone...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Drone armed.')

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    time.sleep(10)'''
def arm_drone(master):
    print('🕹️ Arming drone...')
    
    # Send arm command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Arm command sent. Waiting for confirmation...')
    #time.sleep(2)

    # Confirm drone is armed
    while True:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
        )
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print('✅ Drone confirmed as armed.')
            break
        print('⏳ Waiting for drone to arm...')
        #time.sleep(1)
def takeoff_drone(master, altitude=5):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    print('⏳ Takeoff in progress...')

    time.sleep(10)  # Wait for the drone to reach the altitude
    print(f'✅ Drone has taken off to {altitude} meters.')

# Return to Launch
def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')

# Perform Landing
def land_at_coordinates(drone):
    print('🛬 Sending LAND command to given coordinates...')
    drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, drone.target_system, drone.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
    ))

    drone.mav.command_long_send(
        drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')
    time.sleep(10)
def set_waypoint(master, latitude, longitude, altitude):
    print(f'📍 Sending waypoint to Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}m')

    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,  # Time_boot_ms (set to 0 for now)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000),  # Control position, no yaw or velocity
        int(latitude * 1e7),
        int(longitude * 1e7),
        altitude,
        0, 0, 0,  # Velocity X, Y, Z
        0, 0, 0,  # Acceleration X, Y, Z
        0.0,      # Yaw
        0.0       # Yaw Rate (This was missing)
    ))

    print('✅ Waypoint command sent.')


# Main Function
def main():
   ''' cap = cv2.VideoCapture(VIDEO_SOURCE)
    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()'''
drone = connect_to_drone()
    #if not drone:
       # return

''' while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue

        data, bbox, _ = qr_detector.detectAndDecode(frame)

        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i][0])
                pt2 = tuple(bbox[(i + 1) % len(bbox)][0])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            if data:
                print(f'✅ QR Code detected: {data}')
                cv2.putText(frame, f"QR: {data}", (bbox[0][0][0], bbox[0][0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                land_at_coordinates(drone)
                print('⏳ Waiting for 5 seconds after landing...')
                time.sleep(5)

                print('🔄 Rotating servo forward and back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(2)
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
                time.sleep(2)
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)
                time.sleep(5)'''

set_guided_mode(drone)
                #arm_and_takeoff(drone, altitude=2)
arm_drone(drone)
time.sleep(5)
takeoff_drone(drone, altitude=3)
               # set_waypoint(drone, 10.04902393, 7.3432342, 3)
# break

'''cv2.imshow('QR Code Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if drone:
        drone.close()
        print('🔌 Drone connection closed.')'''

if __name__ == '__main__':
    main()
