from pymavlink import mavutil
import time

# Set COM port, baud rate, and servo channel
COM_PORT = 'COM8'  
BAUD_RATE = 57600  
SERVO_CHANNEL = 9  # Set to channel 1
SERVO_PWM_FORWARD = 1700  # Example PWM for forward motion
SERVO_PWM_CLOSE = 1400
SERVO_PWM_STOP = 1500 

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

def main():
    drone = connect_to_drone()
    if drone is None:
        return

    print('⏳ Waiting for 5 seconds...')
    time.sleep(2)

    print('🔄 Rotating servo 360 degrees forward...')
    # set_servo(drone, SERVO_CHANNEL, SERVO_PWM_FORWARD)
    time.sleep(4)

    print('🔄 Rotating servo 360 degrees back...')
    set_servo(drone, SERVO_CHANNEL, SERVO_PWM_CLOSE)
    time.sleep(2)

    print('Stopping servo...')
    set_servo(drone, SERVO_CHANNEL, SERVO_PWM_STOP)

    drone.close()
    print('🔌 Drone connection closed.')

if __name__ == '__main__':
    main()
