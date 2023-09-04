#connect to mavlink message
"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('tcp:localhost:5762') #
time.sleep(1)

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id,  # The MAVLink message ID
        1e6 / frequency_hz,
        # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0,  # Unused parameters
        0,
        # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# Make sure the connection is valid
master.wait_heartbeat()
time.sleep(1)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 50)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,50)  # Configure ATTITUDE message to be sent at 2Hz
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_AHRS2, 50)

# Get some information !
while True:
    try:
        print(master.recv_match().to_dict())
    except:
        #print('.')
        pass

'''
{'mavpackettype': 'AHRS2', 'roll': 0.008161118254065514, 'pitch': 0.018288079649209976, 'yaw': 0.39535635709762573, 'altitude': 0.25999999046325684, 'lat': 0, 'lng': 0}
{'mavpackettype': 'ATTITUDE', 'time_boot_ms': 74755, 'roll': 0.0063266875222325325, 'pitch': 0.002819434739649296, 'yaw': 0.4023672640323639, 'rollspeed': 0.00012109271483495831, 'pitchspeed': -0.0007827566587366164, 'yawspeed': -4.824780626222491e-05}
{'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.011027774773538113, 'heading': 23, 'throttle': 0, 'alt': 0.0, 'climb': -0.0030034661758691072}
{'mavpackettype': 'AHRS2', 'roll': 0.008064577355980873, 'pitch': 0.018215322867035866, 'yaw': 0.3952871859073639, 'altitude': 0.26999998092651367, 'lat': 0, 'lng': 0}
{'mavpackettype': 'HEARTBEAT', 'type': 2, 'autopilot': 3, 'base_mode': 81, 'custom_mode': 0, 'system_status': 3, 'mavlink_version': 3}
{'mavpackettype': 'GLOBAL_POSITION_INT', 'time_boot_ms': 75003, 'lat': 0, 'lon': 0, 'alt': 0, 'relative_alt': 358, 'vx': 0, 'vy': -1, 'vz': 0, 'hdg': 2305}
{'mavpackettype': 'SYS_STATUS', 'onboard_control_sensors_present': 1467022383, 'onboard_control_sensors_enabled': 1382063151, 'onboard_control_sensors_health': 1466997807, 'load': 551, 'voltage_battery': 656, 'current_battery': 102, 'battery_remaining': 98, 'drop_rate_comm': 0, 'errors_comm': 0, 'errors_count1': 0, 'errors_count2': 0, 'errors_count3': 0, 'errors_count4': 0}
{'mavpackettype': 'POWER_STATUS', 'Vcc': 4832, 'Vservo': 74, 'flags': 4}
{'mavpackettype': 'MEMINFO', 'brkval': 0, 'freemem': 50208, 'freemem32': 50208}
{'mavpackettype': 'NAV_CONTROLLER_OUTPUT', 'nav_roll': 0.00024743095855228603, 'nav_pitch': -0.000325048400554806, 'nav_bearing': 23, 'target_bearing': 0, 'wp_dist': 0, 'alt_error': -0.3230525553226471, 'aspd_error': 0.0, 'xtrack_error': 0.0}
{'mavpackettype': 'MISSION_CURRENT', 'seq': 0}
{'mavpackettype': 'SERVO_OUTPUT_RAW', 'time_usec': 75008266, 'port': 0, 'servo1_raw': 0, 'servo2_raw': 0, 'servo3_raw': 0, 'servo4_raw': 0, 'servo5_raw': 0, 'servo6_raw': 0, 'servo7_raw': 0, 'servo8_raw': 0, 'servo9_raw': 1000, 'servo10_raw': 1000, 'servo11_raw': 1000, 'servo12_raw': 1000, 'servo13_raw': 0, 'servo14_raw': 0, 'servo15_raw': 0, 'servo16_raw': 0}
{'mavpackettype': 'RC_CHANNELS', 'time_boot_ms': 75008, 'chancount': 0, 'chan1_raw': 0, 'chan2_raw': 0, 'chan3_raw': 0, 'chan4_raw': 0, 'chan5_raw': 0, 'chan6_raw': 0, 'chan7_raw': 0, 'chan8_raw': 0, 'chan9_raw': 0, 'chan10_raw': 0, 'chan11_raw': 0, 'chan12_raw': 0, 'chan13_raw': 0, 'chan14_raw': 0, 'chan15_raw': 0, 'chan16_raw': 0, 'chan17_raw': 0, 'chan18_raw': 0, 'rssi': 255}
{'mavpackettype': 'SCALED_IMU2', 'time_boot_ms': 75008, 'xacc': 10, 'yacc': -3, 'zacc': -972, 'xgyro': 3, 'ygyro': 2, 'zgyro': 5, 'xmag': 216, 'ymag': -60, 'zmag': 496, 'temperature': 5115}
{'mavpackettype': 'SCALED_IMU3', 'time_boot_ms': 75008, 'xacc': -14, 'yacc': -4, 'zacc': -995, 'xgyro': 0, 'ygyro': -1, 'zgyro': 0, 'xmag': 290, 'ymag': -67, 'zmag': 509, 'temperature': 4843}
{'mavpackettype': 'SCALED_PRESSURE', 'time_boot_ms': 75008, 'press_abs': 1003.8721923828125, 'press_diff': 0.0, 'temperature': 4956, 'temperature_press_diff': 0}
{'mavpackettype': 'SCALED_PRESSURE2', 'time_boot_ms': 75010, 'press_abs': 1004.3130493164062, 'press_diff': 0.0, 'temperature': 4558, 'temperature_press_diff': 0}
{'mavpackettype': 'GPS_RAW_INT', 'time_usec': 0, 'fix_type': 1, 'lat': 0, 'lon': 0, 'alt': -17000, 'eph': 9999, 'epv': 9999, 'vel': 0, 'cog': 0, 'satellites_visible': 0, 'alt_ellipsoid': 0, 'h_acc': 4294967295, 'v_acc': 3750742272, 'vel_acc': 999000, 'hdg_acc': 0, 'yaw': 0}
{'mavpackettype': 'SYSTEM_TIME', 'time_unix_usec': 0, 'time_boot_ms': 75011}
{'mavpackettype': 'AHRS', 'omegaIx': -0.0009331110632047057, 'omegaIy': -0.00024451687932014465, 'omegaIz': -0.0001950155128724873, 'accel_weight': 0.0, 'renorm_val': 0.0, 'error_rp': 0.002369424793869257, 'error_yaw': 0.0018454761011525989}
{'mavpackettype': 'HWSTATUS', 'Vcc': 4827, 'I2Cerr': 0}
{'mavpackettype': 'TERRAIN_REPORT', 'lat': 0, 'lon': 0, 'spacing': 0, 'terrain_height': 0.0, 'current_height': 0.0, 'pending': 0, 'loaded': 0}
{'mavpackettype': 'BATTERY2', 'voltage': 14214, 'current_battery': 4455}
{'mavpackettype': 'EKF_STATUS_REPORT', 'flags': 167, 'velocity_variance': 0.0, 'pos_horiz_variance': 0.0018435628153383732, 'pos_vert_variance': 0.00652950769290328, 'compass_variance': 0.0037317152600735426, 'terrain_alt_variance': 0.0, 'airspeed_variance': 0.0}
{'mavpackettype': 'VIBRATION', 'time_usec': 75018330, 'vibration_x': 0.02541315369307995, 'vibration_y': 0.03951943293213844, 'vibration_z': 0.04083768650889397, 'clipping_0': 0, 'clipping_1': 0, 'clipping_2': 0}
{'mavpackettype': 'BATTERY_STATUS', 'id': 1, 'battery_function': 0, 'type': 0, 'temperature': 32767, 'voltages': [14214, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535], 'current_battery': 4455, 'current_consumed': 840, 'energy_consumed': 423, 'battery_remaining': 74, 'time_remaining': 0, 'charge_state': 1, 'voltages_ext': [0, 0, 0, 0], 'mode': 0, 'fault_bitmask': 0}
{'mavpackettype': 'ATTITUDE', 'time_boot_ms': 75020, 'roll': 0.006361236795783043, 'pitch': 0.002815251238644123, 'yaw': 0.40241801738739014, 'rollspeed': -0.00041006982792168856, 'pitchspeed': -0.0009842882864177227, 'yawspeed': -3.1888397643342614e-05}
{'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.01188517827540636, 'heading': 23, 'throttle': 0, 'alt': 0.0, 'climb': -0.000920547463465482}
{'mavpackettype': 'AHRS2', 'roll': 0.008048372343182564, 'pitch': 0.018170630559325218, 'yaw': 0.3952699601650238, 'altitude': 0.3499999940395355, 'lat': 0, 'lng': 0}
{'mavpackettype': 'ATTITUDE', 'time_boot_ms': 75255, 'roll': 0.006248928606510162, 'pitch': 0.002805928699672222, 'yaw': 0.4024771749973297, 'rollspeed': -0.00021677423501387239, 'pitchspeed': -0.0007328441133722663, 'yawspeed': -0.0011147067416459322}
{'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.012035394087433815, 'heading': 23, 'throttle': 0, 'alt': 0.0, 'climb': 0.0010876881424337626}
{'mavpackettype': 'AHRS2', 'roll': 0.008002283051609993, 'pitch': 0.018192313611507416, 'yaw': 0.39520660042762756, 'altitude': 0.3799999952316284, 'lat': 0, 'lng': 0}
'''
