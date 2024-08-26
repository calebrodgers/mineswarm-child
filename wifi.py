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
def calculate_counts(distance):
    """ Convert distance to encoder counts """
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

# Move for a target number of counts
def move_straight(target_counts):
    """ Move straight for a certain number of encoder counts """
    encoders.get_counts(reset=True)
    while True:
        current_counts = sum(encoders.get_counts()) / 2
        if current_counts >= target_counts:
            break

        motors.set_speeds(MAX_SPEED, MAX_SPEED)
    motors.off()

def turn_angle(target_angle):
    """ Turn a certain angle using the gyro with PD control, using microseconds for timing """
    current_angle = 0
    last_time = time.ticks_us()
    imu.gyro.read()  # Initial read to ensure data is ready

    # PD Controller coefficients
    Kp = 300  # Proportional gain
    Kd = 50   # Derivative gain
    last_angle = 0  # To store the last angle for derivative calculation

    while True:
        if imu.gyro.data_ready():
            now = time.ticks_us()
            imu.gyro.read()
            turn_rate = imu.gyro.last_reading_dps[2]  # Z-axis
            time_elapsed = time.ticks_diff(now, last_time) / 1000000  # Convert microseconds to seconds
            current_angle += turn_rate * time_elapsed

            # PD control calculation
            error = target_angle - current_angle
            derivative = (current_angle - last_angle) / time_elapsed

            # Check if the target angle is reached within the tolerance
            if abs(error) < ANGLE_TOLERANCE:
                motors.off()
                break

            # Calculate control output
            turn_speed = Kp * error - Kd * derivative
            # Clamp the turn speed to be within the max speed range
            turn_speed = max(-MAX_SPEED, min(turn_speed, MAX_SPEED))
            motors.set_speeds(-turn_speed, turn_speed)

            # Update variables for the next iteration
            last_time = now
            last_angle = current_angle
        else:
            # Small delay to prevent the loop from running too fast
            time.sleep(0.01)

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
    #send_at_command("AT+CWDHCP=0,0")
    #send_at_command("AT+CIPSTA=\"192.168.4.15\",\"192.168.4.1\",\"255.255.255.0\"")

    # Connect to AP
    send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), 20)

    send_at_command("AT+CIFSR",10)

def set_ip(ip, save=False):
    if save == True:
        # Save the updated IP to the ESP8266
        send_at_command(f"AT+CIPSTA_DEF=\"{ip}\"", 2)
    else:
        # Update IP until the next restart
        send_at_command(f"AT+CIPSTA_CUR=\"{ip}\"", 2)

def setup_udp_server(local_port):
    """
    Setup a UDP server on a specified local port to listen for incoming data.
    """
    response = send_at_command("AT+CIPSTART=\"UDP\",\"192.168.4.15\",0,{},2".format(local_port))
    if "OK" in response:
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
    if "OK" in response:
        print("UDP client setup successful. Target IP: {} on port: {}".format(remote_ip, remote_port))
        return True
    else:
        print("Failed to set up UDP client:", response)
        return False

def send_udp_data(data):
    """
    Send data over UDP to the previously specified IP and port.
    """
    length = len(data)
    send_command = 'AT+CIPSEND={}'.format(length)
    response = send_at_command(send_command, 0.5)  # Increase time if needed
    if "OK" or ">" in response:  # Check if ready to receive data
        print("Ready to send data.")
        response = send_at_command(data, 0.5)  # Send the actual data
        if "SEND OK" in response:
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
    if "OK" in response:
        print("Connection closed successfully.")
    else:
        print("Failed to close connection:", response)


def listen_udp(duration=2):
    """
    Listen for incoming UDP data continuously for a specified duration in seconds.
    """
     # Get the current time to start timing
    while True:
        response = uart1.read()
        if response:
            try:
                print("Received:", response.decode('utf-8'))  # Decode response as UTF-8
                command = response.decode('utf-8').strip()
                if "start" in command.lower():
                    return True
            except Exception as e:  # Handle decoding exceptions
                print("Received (raw):", response)  # Print raw response if decoding fails
                print("Decode Error:", e)




# Example setup
# Example setup
#ssid = 'UGV'
#password = '12345678'
#port = 1112

#test
ssid = 'TP-Link_EBC6'
password = '58221471'
port = 1112

# Initialize Wi-Fi
setup_wifi(ssid, password)
close_connection()
# Setup remote details and local listening port
remote_ip = "192.168.4.4"
remote_port = 60000
local_port = 1112  # This port is where your ESP8266 will listen for incoming UDP packets
server_port = 1112


setup_udp_server(server_port)
if listen_udp(10):
    print("Start command received. Beginning navigation...")
    close_connection()
    setup_udp_client('192.168.4.4', 60000, 1112)  # Server IP and port
    # Step 2: Navigate waypoints and send updates
    for distance, theta in waypoints:
        display.fill(0)
        display.text(f"Moving to {distance}m", 0, 0)
        display.text(f"Then turn {theta} degrees", 0, 10)
        display.show()
        encoders.get_counts(reset=True)
        last_counts = 0
        target_counts = calculate_counts(distance)
        while True:
            motors.set_speeds(MAX_SPEED, MAX_SPEED)


            current_counts = sum(encoders.get_counts()) / 2
            counts_increment = current_counts - last_counts

            print(current_counts)
            current_position[0] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.sin(math.radians(real_angle))
            current_position[1] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.cos(math.radians(real_angle))

            print(current_position)
            send_udp_data(f"position {current_position}")
            if current_counts >= target_counts:
                break
            last_counts = current_counts

        motors.off()

        turn_angle(theta)
        real_angle += theta

        # Send update via UDP




    display.text("Waypoint navigation complete", 0, 20)
    display.show()

    # Your function to handle incoming data
 # Close the server connection


# Example of using a single connection for both sending and receiving


# Optionally, listen for incoming UDP data (this should be handled asynchronously or in a loop)
# listen_udp()

# Function to send data (modify IP and port as needed)
#send_udp_data("192.168.1.100", 8888, "Hello ESP8266!")

# Listen for incoming UDP data (this should be handled asynchronously or in a loop)
#listen_udp()

