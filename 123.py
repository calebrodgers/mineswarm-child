

from zumo_2040_robot import robot
import time
import math

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

#waypoints = [[0.4, 0], [0.27, 180]]
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

def generate_path(length, width):
    path = []
    path.append([0, 0])  # 起始点

    # 确定转向角度，正的宽度表示左转，负的宽度表示右转
    direction = 90 if width > 0 else -90

    # 使用绝对值计算需要的垂直线段数量
    num_segments = int(abs(width) // 0.1)
    remaining_width = abs(width) - num_segments * 0.1

    # 生成路径
    for _ in range(num_segments):
        # 前进length
        path.append([length, 0])
        # 转向90度
        path.append([0, direction])
        # 前进0.1
        path.append([0.1, 0])
        # 转向90度
        path.append([0, direction])

        # 改变方向
        direction *= -1

    # 如果有剩余宽度，处理剩余部分
    if remaining_width > 0:
        # 前进length
        path.append([length, 0])
        # 转向90度
        path.append([0, direction])
        # 前进剩余宽度（应该是0.1）
        path.append([remaining_width, 0])
        # 转向90度
        path.append([0, direction])

    # 最后一步返回到起点，转向方向修正
    path.append([0, -direction])
    path.append([abs(width), 0])

    # 再次转向90度，转到出发方向的相反方向
    path.append([0, -direction])  # 保持与起始方向相反

    # 沿着相反方向走一段距离，这里假设是length的1/2（你可以调整这个值）
    path.append([length / 2, 0])

    return path
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
waypoints = generate_path(0.4, 0.6)

i = 0;

####################################################################################################
########testing unload##############################################################################
#main control loop to proceed each waypoint
time.sleep(2)
motors.set_speeds(MAX_SPEED/2, MAX_SPEED/2)
time.sleep(1.5)
motors.off()
# Reset and enable IMU
imu.reset()
imu.enable_default()
time.sleep(1)

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



####################################################################################################
########testing unload##############################################################################
#main control loop to proceed each waypoint
#motors.set_speeds(robot.Motors.MAX_SPEED/3, robot.Motors.MAX_SPEED/3)
#time.sleep(3)
motors.off()
time.sleep(1)
