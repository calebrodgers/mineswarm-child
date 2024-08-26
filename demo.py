

from machine import Pin
from zumo_2040_robot import robot
import time
import math

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

#waypoints = [[0.3, 0], [0.1, 90], [0.3, 90], [0.1, -90], [0.3, -90], [0.1, 90], [0.3, 90], [0.32, 90], [0.05, -90]]
waypoints = [[0.3, 0]]

# 点的定义，例如[ 0.1, -90]，机器人行驶0.1米后转-90°（-90°向右转，90°向左转，0°是不转，走直线
#想加点，就加一个[distance,theta], 注意，是[[],[],[],[]]这种形式，检查最外面的两个中括号
#
current_angle = 0  # accumulate to -90 degree, then turn angle, aim angle is



# Calibration process. Keep it in a horizontal plane for 5 seconds
# 静置5000ms = 5s

last_time = time.ticks_us()
last_angle = 0

aim_angle = 0

# time delay as you need for the Zumo to start move.
time.sleep(1)

# Count the mines (for testing only)
# TODO: Remove this 
mine_count = 0

def callback(pin):
     global mine_count
     mine_count = mine_count + 1	

# Configure interrupt pin 
detector_pin = Pin(28, Pin.IN)
detector_pin.irq(trigger=Pin.IRQ_RISING, handler=callback)

# Counts measurement for Encoder
def calculate_counts(distance):
    """ Convert distance to encoder counts """
    return int((distance / WHEEL_CIRCUMFERENCE) * CPR)

# Turn angle function when the Zumo reachs the destination
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


i = 0

####################################################################################################
########testing unload##############################################################################
#main control loop to proceed each waypoint
"""
time.sleep(2)
motors.set_speeds(MAX_SPEED, MAX_SPEED)
time.sleep(1.5)
motors.off()
# Reset and enable IMU
imu.reset()
imu.enable_default()
time.sleep(1)
"""
#####################################################################################################
#####################################################################################################


#main control loop to proceed each waypoint
for distance, theta in waypoints:
        encoders.get_counts(reset=True)
        last_counts = 0
        target_counts = calculate_counts(distance)

        aim_angle += theta
        #turn_angle(aim_angle)

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
            print('cali')


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

display.fill(0)
display.text(f"Mines found {mine_count}", 0, 0)
display.show()

####################################################################################################
########testing unload##############################################################################
#main control loop to proceed each waypoint
"""
motors.set_speeds(robot.Motors.MAX_SPEED/3, robot.Motors.MAX_SPEED/3)
time.sleep(3)
motors.off()
time.sleep(1)
"""