

from zumo_2040_robot import robot
import time
import math

# Constants
MAX_SPEED = robot.Motors.MAX_SPEED / 2  # Constant speed at max speed/4
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
stationary_gz = 0
# Waypoints list: Each element is [distance (in meters), theta (in degrees)]
waypoints = [[0.1, 0],[0.3, -90], [0.30, -90], [0.10, -90],[0.295,90],[0.19,0],[0.1,0]]
current_angle = 0  # accumulate to -90 degree, then turn angle, aim angle is


calibration_start = time.ticks_ms()
reading_count = 0
while time.ticks_diff(time.ticks_ms(), calibration_start) < 1000:
    if imu.gyro.data_ready():
        imu.gyro.read()
        stationary_gz += imu.gyro.last_reading_dps[2]
        reading_count += 1
stationary_gz /= reading_count
last_time = time.ticks_us()
last_angle = 0

aim_angle = 0


time.sleep(5)
def calculate_counts(distance):
    """ Convert distance to encoder counts """
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

def turn_angle(target_angle):
    """ Turn a certain angle using the gyro with PD control, using microseconds for timing """
    global current_angle
    last_time1 = time.ticks_us()


    # PD Controller coefficients
    Kp = 350  # Proportional gain
    Kd = 7   # Derivative gain
    last_angle1 = current_angle  # To store the last angle for derivative calculation

    while True:
        if imu.gyro.data_ready():
            now1 = time.ticks_us()
            imu.gyro.read()
            turn_rate = imu.gyro.last_reading_dps[2]-stationary_gz  # Z-axis
            time_elapsed = time.ticks_diff(now1, last_time1) / 1000000  # Convert microseconds to seconds
            current_angle += turn_rate * time_elapsed
            print(f"Updated current_angle: {current_angle} with turn_rate: {turn_rate}, time_elapsed: {time_elapsed}")

            # PD control calculation
            error = target_angle - current_angle
            derivative = (current_angle - last_angle1) / time_elapsed

            # Check if the target angle is reached within the tolerance
            if abs(error) < ANGLE_TOLERANCE:
                motors.off()
                print(f"Final angle after turn: {current_angle}")
                return

            # Calculate control output
            turn_speed = Kp * error - Kd * turn_rate
            # Clamp the turn speed to be within the max speed range
            turn_speed = max(-robot.Motors.MAX_SPEED, min(turn_speed, robot.Motors.MAX_SPEED))
            motors.set_speeds(-turn_speed, turn_speed)

            # Update variables for the next iteration
            last_time1 = now1
            last_angle1 = current_angle
        else:
            # Small delay to prevent the loop from running too fast
            time.sleep(0.01)

# Initialize UART1 with GPIO pins (specific to your microcontroller board layout)



 # Close the server connection






#main control loop to proceed each waypoint
for distance, theta in waypoints:
        encoders.get_counts(reset=True)
        last_counts = 0
        target_counts = calculate_counts(distance)




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
                    print(f"normal current_angle: {current_angle} with turn_rate: {turn_rate}, time_elapsed: {time_elapsed}")


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
        time.sleep(2)
        print('next')
        turn_angle(aim_angle)
        print('current'+str(current_angle))

        real_angle += theta
        print('real'+str(real_angle))
        aim_angle += theta
        print('aim'+str(aim_angle))
