from machine import Pin
from libraries import testing as file
from zumo_2040_robot import robot
import time
import math

display = robot.Display()

pin = Pin(28, Pin.IN)

isMine = False

num = file.test(2, 3)

def callback(pin):
	global isMine
	isMine = ~isMine

pin.irq(trigger=Pin.IRQ_RISING, handler=callback)

while True:
	if isMine:
		display.fill(0)
		display.text("No mine", 0, 0)
		display.text(f"Num {num}", 0, 10)
		display.show()
	else:
		display.fill(0)
		display.text("Mine", 0, 0)
		display.show()