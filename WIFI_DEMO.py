# Write your code here :-)
# Write your code here :-)
#
from machine import UART, Pin

from zumo_2040_robot import robot
import time
import math

# Constants
MAX_SPEED = robot.Motors.MAX_SPEED / 4  # Constant speed at max speed/4
WHEEL_DIAMETER = 0.039  # meters (example value)
CPR = 1200  # Counts per revolution (adjust according to your encoder specs)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
ANGLE_TOLERANCE = 3  # degrees, tolerance for gyro angle accuracy

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

# Waypoints list: Each element is [distance (in meters), theta (in degrees)]
waypoints = [[0.40, -90], [0.20, -90], [0.40, 90],[0.20,90]]

# Get the number of encoder counts to a given distance

# Initialize UART1 with GPIO pins (specific to your microcontroller board layout)
uart1 = UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))
#

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
    time.sleep(1)  # 等待一段时间确保指令已发送并且模块已复位

# 调用函数重置 ESP8266


def setup_wifi(ssid, password):
    # Test AT command
    send_at_command("AT+RST", 5)  # reset ESP8266
    send_at_command("ATE0")       # disable echo
    send_at_command("AT")
    # Set mode to STA (Station)
    send_at_command("AT+CWMODE=1")

    # Configure the IP
    send_at_command("AT+CWDHCP=0,0")
    send_at_command("AT+CIPSTA=\"192.168.0.11\",\"192.168.0.1\",\"255.255.255.0\"")

    # Connect to AP
    send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), 20)

    send_at_command("AT+CIFSR",10)








# Example setup
# Example setup
#ssid = 'UGV'
#password = '12345678'
#port = 1112

#test
ssid = 'TP-Link_EBC6'
password = '58221471'


# Initialize Wi-Fi
setup_wifi(ssid, password)

