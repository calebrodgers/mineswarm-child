# This example provides an interface to test the motors and encoders.
#
# Holding button A or C causes the left or right motor to accelerate;
# releasing the button causes the motor to decelerate. Tapping the button
# while the motor is not running reverses the direction it runs.
#
# Encoder counts are displayed on the bottom two lines of the OLED.

from zumo_2040_robot import robot
import time

display = robot.Display()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
motors = robot.Motors()
encoders = robot.Encoders()

display.text("Hold=run", 32, 0)
display.text("Tap=flip", 32, 8)
display.text("A", 8, 28)
display.text("C", 112, 28)
display.text("L: ", 24, 48)
display.text("R: ", 24, 56)

arrows = ["v", None, "^"]
left_dir = 1
right_dir = 1
left_speed = 0
right_speed = 0
last_update_time = 0
button_count_a = 0
button_count_c = 0

while True:
    # Check if button A is pressed
    if button_a.is_pressed():
        left_speed = motors.MAX_SPEED/5
        right_speed = motors.MAX_SPEED/5
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < 2000:
            motors.set_speeds(left_speed, right_speed)
            display.fill_rect(40, 48, 64, 16, 0)
            left_encoder, right_encoder = encoders.get_counts()
            display.text(f"{left_encoder:>8}", 40, 48)
            display.text(f"{right_encoder:>8}", 40, 56)
            display.show()
        # Stop the motors after 5 seconds
        motors.set_speeds(0, 0)

    # Check if button C is pressed
    if button_c.is_pressed():
        left_speed = -motors.MAX_SPEED/5
        right_speed = -motors.MAX_SPEED/5
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < 2000:
            motors.set_speeds(left_speed, right_speed)
            display.fill_rect(40, 48, 64, 16, 0)
            left_encoder, right_encoder = encoders.get_counts()
            display.text(f"{left_encoder:>8}", 40, 48)
            display.text(f"{right_encoder:>8}", 40, 56)
            display.show()
        # Stop the motors after 5 seconds
        motors.set_speeds(0, 0)

    time.sleep(0.1)  # Add a small delay to prevent busy waiting