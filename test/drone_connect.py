from pymavlink import mavutil

def connect_to_drone(connection_string='udp:127.0.0.1:14550'):
    print('Connecting to drone...')
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print('Drone connected')
    return master

if __name__ == '__main__':
    connect_to_drone()
