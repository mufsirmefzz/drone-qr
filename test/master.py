master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    11,    # Servo channel
    2000,  # PWM value
    0, 0, 0, 0, 0
)
