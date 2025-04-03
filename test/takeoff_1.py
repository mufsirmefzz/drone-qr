import time
from pymavlink import mavutil

COM_PORT = 'COM8'
BAUD_RATE = 57600

def connect_to_drone():
    print(f'🔌 Connecting to drone on {COM_PORT}...')
    try:
        master = mavutil.mavlink_connection(COM_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print(f'✅ Connected. Heartbeat from system {master.target_system}, component {master.target_component}')
        return master
    except Exception as e:
        print(f'❌ Connection failed: {e}')
        return None

def check_pre_arm(master):
    print("🔍 Checking for pre-arm errors...")
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        print(f"📢 {msg.text}")
        if "PreArm" not in msg.text:
            break

def set_guided_mode(master):
    print('🛫 Setting mode to GUIDED...')
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4
    )
    time.sleep(2)
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    print(f'✅ Mode set to GUIDED (Mode: {msg.custom_mode})')

def arm_drone(master):
    print('🕹️ Arming drone...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)

    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print('✅ Drone armed.')
    else:
        print('❌ Drone did not arm.')

def takeoff_drone(master, altitude=5):
    print(f'🚀 Taking off to {altitude} meters...')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, altitude, 0
    )
    time.sleep(10)

    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(f'📍 Current altitude: {msg.relative_alt / 1000.0} meters')

def main():
    drone = connect_to_drone()
    if not drone:
        return
    
    check_pre_arm(drone)
    set_guided_mode(drone)
    arm_drone(drone)
    time.sleep(5)
    takeoff_drone(drone, altitude=3)

if __name__ == '__main__':
    main()
