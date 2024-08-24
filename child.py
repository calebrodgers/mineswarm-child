from machine import UART, Pin
from libraries import esp8266 as wifi
from zumo_2040_robot import robot
import time
import math

# Constants
MAX_SPEED = robot.Motors.MAX_SPEED / 2  # Maximum speed divided by 2
WHEEL_DIAMETER = 0.039  # Wheel diameter in meters
CPR = 1200  # Counts per revolution for the encoder
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER  # Circumference of the wheel
ANGLE_TOLERANCE = 2  # Tolerance for gyro angle accuracy in degrees

# Robot ID
robot_id = 1

# Initialize components
motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()
display = robot.Display()

# Reset and enable IMU
imu.reset()
imu.enable_default()

# Initial position and angle
current_position = [0, 0]
real_angle = 0
stationary_gz = 0

# Wi-Fi connection status
isConnected = False

# Interrupt handler for mine detection
pin = Pin(28, Pin.IN)

def callback(pin):
    if isConnected:
        wifi.send_udp_data("Mine identified", uart1)

pin.irq(trigger=Pin.IRQ_RISING, handler=callback)

# Waypoints list: Each element is [distance (in meters), theta (in degrees)]
# The robot always keeps its heading angle at zero.
waypoints = [[0.2, 0], [0.2, 180]]

current_angle = 0  # Accumulated angle
aim_angle = 0  # Target angle

# Calibration process: keep the robot on a horizontal plane for 5 seconds
last_time = time.ticks_us()
last_angle = 0

# Wi-Fi setup
uart1 = UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))

def generate_path(length, width):
    """Generate a path based on the given length and width."""
    path = []
    path.append([0, 0])  # Starting point

    direction = 90 if width > 0 else -90  # Determine turning direction

    num_segments = int(abs(width) // 0.1)
    remaining_width = abs(width) - num_segments * 0.1

    for _ in range(num_segments):
        path.append([length, 0])  # Move forward by 'length'
        path.append([0, direction])  # Turn by 90 degrees
        path.append([0.1, 0])  # Move forward by 0.1 meters
        path.append([0, direction])  # Turn by 90 degrees
        direction *= -1  # Change direction

    if remaining_width > 0:
        path.append([length, 0])  # Move forward by 'length'
        path.append([0, direction])  # Turn by 90 degrees
        path.append([remaining_width, 0])  # Move forward by remaining width
        path.append([0, direction])  # Turn by 90 degrees

    path.append([0, -direction])  # Correct direction to face the starting point
    path.append([abs(width), 0])  # Move forward by the width of the path
    path.append([0, -direction])  # Turn 90 degrees to face the opposite direction
    path.append([length / 2, 0])  # Move half the length to the starting position

    return path

def calculate_counts(distance):
    """Convert distance to encoder counts."""
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

def turn_angle(target_angle):
    """Turn the robot to a specific angle using the gyro with PD control."""
    global current_angle
    last_time1 = time.ticks_us()

    # PD Controller coefficients
    Kp = 350  # Proportional gain
    Kd = 7  # Derivative gain
    last_angle1 = current_angle  # Store last angle for derivative calculation

    while True:
        if imu.gyro.data_ready():
            now1 = time.ticks_us()
            imu.gyro.read()
            turn_rate = imu.gyro.last_reading_dps[2] - stationary_gz  # Z-axis
            time_elapsed = time.ticks_diff(now1, last_time1) / 1000000  # Convert to seconds
            current_angle += turn_rate * time_elapsed

            # PD control calculation
            error = target_angle - current_angle
            derivative = (current_angle - last_angle1) / time_elapsed

            # Check if the target angle is within tolerance
            if abs(error) < ANGLE_TOLERANCE:
                motors.off()
                print(f"Final angle after turn: {current_angle}")
                return

            # Calculate control output
            turn_speed = Kp * error - Kd * turn_rate
            turn_speed = max(-robot.Motors.MAX_SPEED, min(turn_speed, robot.Motors.MAX_SPEED))
            motors.set_speeds(-turn_speed, turn_speed)

            # Update variables for the next iteration
            last_time1 = now1
            last_angle1 = current_angle
        else:
            time.sleep(0.01)  # Small delay to prevent excessive loop speed

# Wi-Fi configuration
ssid = 'TP-Link_EBC6'
password = '58221471'
local_ip = "192.168.0.15"
local_port = 1112
remote_ip = "192.168.0.107"
remote_port = 50000

# Display Wi-Fi connection status
display.fill(0)
display.text("Connect to Wi-Fi:", 0, 0)
display.show()

# Initialize Wi-Fi
wifi.setup_wifi(ssid, password, local_ip, uart1)
display.fill(0)
display.text("Connected to Wi-Fi:", 0, 0)
display.show()

# Establish UDP client
wifi.setup_udp_client(remote_ip, remote_port, uart1)
time.sleep(1)

# Setup UDP server
if wifi.setup_udp_server(local_ip, local_port, uart1):
    wifi.send_udp_data(f"Child {robot_id} ready", uart1)
    isConnected = True

    while True:
        # Wait for area clearing command via UDP
        print("Waiting for commands...")
        area = wifi.listen_udp(uart1)

        dimensions = area.split(",")
        area_width = float(dimensions[0])
        area_height = float(dimensions[1])

        display.fill(0)
        display.text(str(len(waypoints)), 0, 0)
        display.text(str(area_width), 0, 15)
        display.text(str(area_height), 0, 30)
        display.show()

        print("Width: ", area_width)
        print("Height: ", area_height)

        waypoints = generate_path(area_width, area_height)
        print(waypoints)

        i = 0

        # Initial setup and calibration
        time.sleep(2)
        motors.set_speeds(MAX_SPEED / 2, MAX_SPEED / 2)
        time.sleep(1.5)
        motors.off()

        imu.reset()
        imu.enable_default()
        time.sleep(1)

        # Main control loop to follow each waypoint
        for distance, theta in waypoints:
            encoders.get_counts(reset=True)
            last_counts = 0
            target_counts = calculate_counts(distance)

            aim_angle += theta

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
                print('Calibration complete')

            # PD Controller coefficients
            Kp = 350  # Proportional gain
            Kd = 7  # Derivative gain

            while True:
                current_counts = sum(encoders.get_counts()) / 2
                counts_increment = current_counts - last_counts

                current_position[0] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.sin(math.radians(real_angle))
                current_position[1] += (counts_increment / CPR) * WHEEL_CIRCUMFERENCE * math.cos(math.radians(real_angle))

                if current_counts >= target_counts:
                    break
                last_counts = current_counts

                if imu.gyro.data_ready():
                    now = time.ticks_us()
                    imu.gyro.read()
                    turn_rate = imu.gyro.last_reading_dps[2] - stationary_gz
                    time_elapsed = time.ticks_diff(now, last_time) / 1000000
                    current_angle += turn_rate * time_elapsed

                    error = aim_angle - current_angle
                    derivative = (current_angle - last_angle) / time_elapsed

                    if abs(error) < ANGLE_TOLERANCE:
                        motors.set_speeds(MAX_SPEED, MAX_SPEED)
                    else:
                        turn_speed = Kp * error - Kd * turn_rate
                        turn_speed = max(-robot.Motors.MAX_SPEED, min(turn_speed, robot.Motors.MAX_SPEED))
                        motors.set_speeds(-turn_speed, turn_speed)

                    last_time = now
                    last_angle = current_angle

            motors.off()
            time.sleep(0.1)

            i += 1

        motors.off()
        wifi.send_udp_data(f"Child {robot_id} completed", uart1)
else:
    display.fill(0)
    display.text("Wi-Fi Connection Failed", 0, 0)
    display.show()