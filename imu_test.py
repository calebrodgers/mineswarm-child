# Runs all the IMU sensors, displaying values on the screen and
# printing them to the USB serial port.

from machine import UART, Pin
from zumo_2040_robot import robot
import time
import math
#CONTRAL
##############################################################################################################
##############################################################################################################
# Constants
MAX_SPEED = robot.Motors.MAX_SPEED/4   # Constant speed at max speed/4
WHEEL_DIAMETER = 0.039  # meters (example value)
CPR = 1200  # Counts per revolution (adjust according to your encoder specs)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
ANGLE_TOLERANCE = 2  # degrees, tolerance for gyro angle accuracy

# Initialize components
motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()
display = robot.Display()

# Reset and enable IMU
imu.reset()
imu.enable_default()

current_position = [0, 0]
real_angle = 0
stationary_gz = 0

# Waypoints list: Each element is [distance (in meters), theta (in degrees)]
# waypoint is defined as [distance, theta], distance is the distance to be theta, theta is the angle to be turned when it reach the defined
# distance. The Zumo always keeps its heading angle to be zero.
# 点的定义，例如[ 0.1, -90]，机器人行驶0.1米后转-90°（-90°向右转，90°向左转，0°是不转，走直线
#想加点，就加一个[distance,theta], 注意，是[[],[],[],[]]这种形式，检查最外面的两个中括号
waypoints = [[0.2, 0], [0.2, 90]]

current_angle = 0  # accumulate to -90 degree, then turn angle, aim angle is
aim_angle = 0

# Calibration process. Keep it in a horizontal plane for 5 seconds
# 静置5000ms = 5s
last_time = time.ticks_us()
last_angle = 0
uart1 = UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))

import time
def send_at_command(cmd, wait_time=1):
    print("Sending:", cmd)
    cmd += '\r\n'  # Append carriage return and newline to the command
    uart1.write(cmd.encode('utf-8'))  # Encode the command as UTF-8 before sending
    time.sleep(wait_time)
    response = uart1.read()
    if response:
        try:
            print("Received:", response.decode('utf-8'))  # Try to decode response as UTF-8
        except Exception as e:  # Catch any exception during decoding
            print("Received (raw):", response)  # Print raw response if decoding fails
            print("Decode Error:", e)  # Print error message
    return response


def reset_esp8266():
    uart1.write("AT+RESTORE\r\n")
    time.sleep(1)  # Wait to ensure command is sent and module is reset


def setup_wifi(ssid, password):
    # Test AT command
    send_at_command("AT+RST", 5)  # Reset ESP8266
    send_at_command("ATE0")       # Disable echo
    send_at_command("AT")         # Test AT command
    # Set mode to STA (Station)
    send_at_command("AT+CWMODE=1")
    send_at_command("AT+CWDHCP=0,0")
    #固定小机器人的ip
    send_at_command("AT+CIPSTA=\"192.168.0.15\",\"192.168.0.1\",\"255.255.255.0\"")

    # Connect to AP
    send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), 10)
    send_at_command("AT+CIFSR", 5)


def setup_udp_server(local_port):
    """
    Setup a UDP server on a specified local port to listen for incoming data.
    """
    response = send_at_command("AT+CIPSTART=\"UDP\",\"192.168.0.15\",0,{},2".format(local_port))
    if "OK" in response.decode('utf-8'):
        print("UDP server setup successful on local port: {}".format(local_port))
        return True
    else:
        print("Failed to set up UDP server:", response)
        return False


def setup_udp_client(remote_ip, remote_port, local_port):
    """
    Setup UDP client to send data to a specific remote IP and port.
    """
    command = 'AT+CIPSTART="UDP","{}",{},{}'.format(remote_ip, remote_port, local_port)
    response = send_at_command(command)
    if "OK" in response.decode('utf-8'):
        print("UDP client setup successful. Target IP: {} on port: {}".format(remote_ip, remote_port))
        return True
    else:
        print("Failed to set up UDP client:", response)
        return False


def send_udp_data(data):
    length = len(data)
    send_command = 'AT+CIPSEND={}'.format(length)
    response = send_at_command(send_command, 0.05)  # Increase time if needed
    if ">" in response.decode('utf-8'):  # Check if ready to receive data
        print("Ready to send data.")
        response = send_at_command(data, 0.05)  # Send the actual data
        if "SEND OK" in response.decode('utf-8'):
            print("Data sent successfully")
        else:
            print("Failed to send data:", response)
    else:
        print("Failed to initiate send:", response)


def close_connection():
    """
    Close the current connection.
    """
    print("Closing any existing connection...")
    response = send_at_command("AT+CIPCLOSE")
    if "OK" in response.decode('utf-8') or "ERROR" in response.decode('utf-8'):
        print("Connection closed successfully or no active connection.")
    else:
        print("Failed to close connection:", response)
def format_position_data(x, y):
    """Format the position data into a string."""
    return "Position: x={:.5f}, y={:.5f}".format(x, y)

def calculate_counts(distance):
    """ Convert distance to encoder counts """
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

def update_position(current_counts, current_angle):
    """更新机器人的位置基于编码器和陀螺仪的数据"""
    global current_position
    distance = (current_counts / CPR) * WHEEL_CIRCUMFERENCE
    current_angle_radius = math.radians(current_angle)
    # 使用 current_angle 更新位置
    # 注意这里假设 current_angle 已经是以弧度为单位，如果是度数，需要转换为弧度
    current_position[0] += distance * math.cos(current_angle)
    current_position[1] += distance * math.sin(current_angle)
waypoints = [[0.2, 0], [0.2, 90]]
ssid = 'TP-Link_EBC6'
password = '58221471'
local_port = 1112
remote_ip = "192.168.0.100"
remote_port = 50000
display.fill(0)
display.text("connect to wifi:", 0, 0)
display.show()
# Initialize Wi-Fi
setup_wifi(ssid, password)

display.fill(0)
display.text("connected to wifi:", 0, 0)
display.show()




calibration_start = time.ticks_ms()
calibration_time = 1000
reading_count = 0
while time.ticks_diff(time.ticks_ms(), calibration_start) < calibration_time:
    if imu.gyro.data_ready():
        imu.gyro.read()
        stationary_gz += imu.gyro.last_reading_dps[2]
        reading_count += 1
stationary_gz /= reading_count
setup_udp_client(remote_ip, remote_port, local_port)
for distance, theta in waypoints:
                        encoders.get_counts(reset=True)
                        last_counts = 0
                        target_counts = calculate_counts(distance)
                        display.text(str(target_counts), 0, 45)
                        display.show()

                        aim_angle += theta
                        #turn_angle(aim_angle)




                        # PD Controller coefficients
                        Kp = 50  # Proportional gain
                        Kd = 7   # Derivative gain
                        # To store the last angle for derivative calculation
                        while True:
                            #motors.set_speeds(MAX_SPEED, MAX_SPEED)
                            current_counts = sum(encoders.get_counts()) / 2
                            counts_increment = current_counts - last_counts



                            # reach the wapoint and proceed the next waypoint
                            if current_counts >= target_counts:
                                break
                            last_counts = current_counts



                            # Feedback control of the Zumo when it moves straight
                            if imu.gyro.data_ready():
                                    now = time.ticks_us()
                                    imu.gyro.read()
                                    turn_rate = imu.gyro.last_reading_dps[2]-stationary_gz  # Z-axis
                                    time_elapsed = time.ticks_diff(now, last_time) / 1000000  # Convert microseconds to seconds
                                    current_angle += turn_rate * time_elapsed
                                    current_position[0] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.sin(math.radians(current_angle))
                                    current_position[1] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.cos(math.radians(current_angle))
                                    message = format_position_data(current_position[0],current_position[1])

                                    error = aim_angle - current_angle
                                    derivative = (current_angle - last_angle) / time_elapsed
                                    print("current")
                                    print(current_angle)
                                    print("error")
                                    print(error)

                                    # Check if the target angle is reached within the tolerance
                                    if abs(error) < ANGLE_TOLERANCE:

                                        motors.set_speeds(MAX_SPEED, MAX_SPEED)


                                    else:
                                    # Calculate control output
                                        turn_speed = Kp * error- Kd * turn_rate
                                        # Clamp the turn speed to be within the max speed range
                                        turn_speed = max(-robot.Motors.MAX_SPEED, min(turn_speed, robot.Motors.MAX_SPEED))
                                        motors.set_speeds(-turn_speed, turn_speed)


                                    # Update variables for the next iteration
                                    last_time = now
                                    last_angle = current_angle
                            send_udp_data(message)

                        motors.off()



