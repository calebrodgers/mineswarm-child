# Write your code here :-)
from machine import UART, Pin
from zumo_2040_robot import robot
import time
import math
#CONTRAL
##############################################################################################################
##############################################################################################################
# Constants
MAX_SPEED = robot.Motors.MAX_SPEED/2   # Constant speed at max speed/4
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
waypoints = [[0.2, 0], [0.2, 180]]

current_angle = 0  # accumulate to -90 degree, then turn angle, aim angle is
aim_angle = 0

# Calibration process. Keep it in a horizontal plane for 5 seconds
# 静置5000ms = 5s
last_time = time.ticks_us()
last_angle = 0

# time delay as you need for the Zumo to start move.


# Counts measurement for Encoder
def calculate_counts(distance):
    """ Convert distance to encoder counts """
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

#WIFI
##############################################################################################################
##############################################################################################################
# Initialize UART1 with GPIO pins (specific to your microcontroller board layout)
uart1 = UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))


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
    send_at_command("AT+CIPSTA=\"192.168.0.35\",\"192.168.0.1\",\"255.255.255.0\"")

    # Connect to AP
    send_at_command("AT+CWJAP=\"{}\",\"{}\"".format(ssid, password), 20)
    send_at_command("AT+CIFSR", 10)


def setup_udp_server(local_port):
    """
    Setup a UDP server on a specified local port to listen for incoming data.
    """
    response = send_at_command("AT+CIPSTART=\"UDP\",\"192.168.0.35\",0,{},2".format(local_port))
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


def send_udp_data(data, remote_ip, remote_port):
    """
    command = 'AT+CIPSTART="UDP","{}",{},{}'.format(remote_ip, remote_port, 1112)
    response = send_at_command(command)
    if "OK" in response.decode('utf-8'):
        print("UDP client setup successful. Target IP: {} on port: {}".format(remote_ip, remote_port))
    else:
        print("Failed to set up UDP client:", response)
        return False
    """
    """
    Send data over UDP to the previously specified IP and port.
    """
    length = len(data)
    send_command = 'AT+CIPSEND={}'.format(length)
    response = send_at_command(send_command, 0.5)  # Increase time if needed
    if ">" in response.decode('utf-8'):  # Check if ready to receive data
        print("Ready to send data.")
        response = send_at_command(data, 0.5)  # Send the actual data
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


def listen_udp():
    """
    Listen for incoming UDP data continuously for a specified duration in seconds.
    """

    while True:
        response = uart1.read()
        if response:
            try:
                decoded_response = response.decode('utf-8')
                if "+IPD" in decoded_response:
                    parts = decoded_response.split(':')
                    if len(parts) > 1:
                        data = parts[1].strip()
                        print(data)
                        return data
                return decoded_response.strip()
            except Exception as e:  # Handle decoding exceptions
                print("Received (raw):", response)  # Print raw response if decoding fails
                print("Decode Error:", e)
        
    return None


def plan_path(width, height, robot_width):
    print()
    """
    Constructs a set of waypoints to navigate a rectangular area
    """
    waypoints = []

	# Compute the necessary number of waypoints
    waypoint_count = 2 * int(abs(width) / robot_width) - 2
    print(waypoint_count)
	# Get the first waypoint
    new_waypoint = [height, 0]
    waypoints.append(new_waypoint)

	# Return in the case there is only one waypoint
    if waypoint_count <= 1:
        return waypoints
	
    # Modifies the sign of the angles in the waypoints based on the area width
    angle_sign = 1 if width > 0 else -1

	# Get the remaining "pairs" of waypoints
    for i in range(0, int((waypoint_count / 2))):
        if(i % 2 == 0):
            waypoints.append([robot_width, angle_sign * 90])
            waypoints.append([height, angle_sign * 90])
        else:
            waypoints.append([robot_width, angle_sign * -90])
            waypoints.append([height, angle_sign * -90])

    print(waypoints)
    
	# Return the waypoints to the caller
    return waypoints


 # Declare globals for shared variables
ssid = 'TP-Link_EBC6'
password = '58221471'
local_port = 1112
remote_ip = "192.168.0.107"
remote_port = 50000
display.fill(0)
display.text("connect to wifi:", 0, 0)
display.show()
# Initialize Wi-Fi
setup_wifi(ssid, password)
display.fill(0)
display.text("connected to wifi:", 0, 0)
display.show()
#close_connection()

# Setup UDP server
if setup_udp_server(local_port):
    while True:
        command = listen_udp()
        command = "2"
        print("receive message from ubuntu:")
        setup_udp_client(remote_ip, remote_port, local_port)
        print(command)
        if command:
            # Send the udp data
            send_udp_data("Child executing task...", remote_ip, remote_port)

            if command == "1":
                print("Executing Task 1")
                # Execute Task 1
                motors.set_speeds(robot.Motors.MAX_SPEED/3, robot.Motors.MAX_SPEED/3)
                time.sleep(3)
                motors.off()
                motors.set_speeds(-robot.Motors.MAX_SPEED/3, -robot.Motors.MAX_SPEED/3)
                time.sleep(3)
                motors.off()
                close_connection()
                send_udp_data("Task 1 executed", remote_ip, remote_port)
            elif command == "2":
                print("Executing Task 2")

                # Get the area to clear
                area = listen_udp()

                dimensions = area.split(",")
                area_width = float(dimensions[0])
                area_height = float(dimensions[1])
                
                display.fill(0)
                display.text(str(len(waypoints)), 0, 0)
                display.text(str(area_width), 0, 15)
                display.text(str(area_height), 0, 30)
                display.show()

                print("Wideness: ", area_width)
                print("Highness: ", area_height)
              
                waypoints = plan_path(area_width, area_height, 0.098)
                
                print(waypoints)

                time.sleep(1)

                # Execute Task 2
                """
                for distance, theta in waypoints:
                        encoders.get_counts(reset=True)
                        last_counts = 0
                        target_counts = calculate_counts(distance)
                        display.text(str(target_counts), 0, 45)
                        display.show()

                        aim_angle += theta
                        #turn_angle(aim_angle)
                        
                        i = 1 # Flag for calibration

                        if i == 1:
                            calibration_start = time.ticks_ms()
                            calibration_time = 1000
                            reading_count = 0
                            while time.ticks_diff(time.ticks_ms(), calibration_start) < calibration_time:
                                if imu.gyro.data_ready():
                                    imu.gyro.read()
                                    stationary_gz += imu.gyro.last_reading_dps[2]
                                    reading_count += 1
                            stationary_gz /= reading_count


                        # PD Controller coefficients
                        Kp = 350  # Proportional gain
                        Kd = 7   # Derivative gain
                        # To store the last angle for derivative calculation
                        while True:
                            #motors.set_speeds(MAX_SPEED, MAX_SPEED)
                            current_counts = sum(encoders.get_counts()) / 2
                            counts_increment = current_counts - last_counts

                            current_position[0] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.sin(math.radians(real_angle))
                            current_position[1] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.cos(math.radians(real_angle))

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


                                    # PD control calculation
                                    error = aim_angle - current_angle
                                    derivative = (current_angle - last_angle) / time_elapsed


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


                        motors.off()
                        time.sleep(0.1)
                        #print('next')
                        
                        i += 1
                        #print('real'+str(real_angle))
                        send_udp_data("yeet.", remote_ip, remote_port)
                #data = "" + str(distance) + "," + str(theta)
                #send_udp_data(data, remote_ip, remote_port)
                """
                send_udp_data("Task 2 executed", remote_ip, remote_port)
                close_connection()
            # Add more elif blocks for additional commands if needed
            close_connection()
            setup_udp_server(local_port)  # Reinitialize server after task execution"