import cv2
import numpy as np
from pymavlink import mavutil
import time

# Set COM port, baud rate, and servo channel
COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 9  # Specified channel 11
SERVO_PWM_FORWARD = 2200  # Example PWM for forward motion
SERVO_PWM_CLOSE = 1100  # Example PWM for closing
SERVO_PWM_STOP = 1500
LATITUDE = 10.0524451  # Example latitude
LONGITUDE = 76.620354  # Example longitude


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


def set_servo(master, channel, pwm_value):
    print(f'🔧 Moving servo on channel {channel} to {pwm_value} PWM')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    print('✅ Servo moved.')


def wait_for_landing(master):
    print('⏳ Waiting for drone to land...')
    while True:
        msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if msg and msg.landed_state == 1:  # MAV_LANDED_STATE_ON_GROUND
            print('✅ Drone has landed.')
            break

def land_at_coordinates(master):
    print('🛬 Sending LAND command to given coordinates...')
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
    ))

    master.mav.command_long_send(
        master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ Land command sent.')
    time.sleep(10)

def set_guided_mode(master):
    print('🛫 Switching to GUIDED mode...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 represents GUIDED mode
    )
    print('✅ Mode set to GUIDED.')

def return_to_launch(master):
    print('🚀 Sending Return to Launch (RTL) command...')
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print('✅ RTL command sent.')
def auto_exposure_adjustment(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    avg_brightness = np.mean(gray)
    
    # Adjust exposure based on brightness
    if avg_brightness < 50:
        exposure_value = -4  # Dark conditions
    elif avg_brightness > 200:
        exposure_value = -10  # Bright conditions
    else:
        exposure_value = -6  # Normal

    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Set manual exposure mode
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure_value)

    return frame

def arm_and_takeoff(master, altitude=5):
    print('🕹️ Arming drone...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print('✅ Drone armed.')

    time.sleep(5)

    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    time.sleep(10)

def main():
    video_source = 'rtsp://192.168.144.25:8554/main.264'
    cap = cv2.VideoCapture(video_source)

    if not cap.isOpened():
        print('❌ Error: Unable to open video stream.')
        return

    qr_detector = cv2.QRCodeDetector()
    drone = connect_to_drone()

    while True:
        ret, frame = cap.read()
        if not ret:
            print('❌ Failed to read frame')
            continue
        
        frame = auto_exposure_adjustment(frame)
        
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

                print('🛬 Sending LAND command to given coordinates...')
                drone.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                    0, drone.target_system, drone.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    int(0b110111111000), int(LATITUDE * 1e7), int(LONGITUDE * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0
                ))

                drone.mav.command_long_send(
                    drone.target_system,
                    drone.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                print('✅ Land command sent.')
                time.sleep(10)

                # wait_for_landing(drone)

                print('⏳ Waiting for 5 seconds after landing...')
                time.sleep(5)

                print('🔄 Rotating servo 360 degrees forward...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
                time.sleep(2)

                print('🔄 Rotating servo 360 degrees back...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
                time.sleep(2)

                print(' Servo Stopping...')
                set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)

                time.sleep(5)

                set_guided_mode(drone)

                arm_and_takeoff(drone, altitude=3)

                land_at_coordinates(drone)

                
               # return_to_launch(drone)
                break

        cv2.imshow('QR Code Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if drone:
        drone.close()
        print('🔌 Drone connection closed.')

if __name__ == '__main__':
    main()
