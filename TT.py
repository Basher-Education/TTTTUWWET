import time
import threading
import atexit
import sys
import termios
import contextlib
import os
from RpiMotorLib import RpiMotorLib  # Import RpiMotorLib for DRV8825 control
import RPi.GPIO as GPIO

# Constants for DRV8825 motor control
DIR_PIN_X = 20  # GPIO pin numbers; replace as needed
STEP_PIN_X = 21
ENABLE_PIN_X = 16

DIR_PIN_Y = 19
STEP_PIN_Y = 26
ENABLE_PIN_Y = 13

# Initialize motors using RpiMotorLib
motor_x = RpiMotorLib.A4988Nema(DIR_PIN_X, STEP_PIN_X, (0, 0, 0), "DRV8825")
motor_y = RpiMotorLib.A4988Nema(DIR_PIN_Y, STEP_PIN_Y, (0, 0, 0), "DRV8825")

# GPIO setup for enabling/disabling motors
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENABLE_PIN_X, GPIO.OUT)
GPIO.setup(ENABLE_PIN_Y, GPIO.OUT)

# Helper function to enable or disable motors
def enable_motors(enable=True):
    GPIO.output(ENABLE_PIN_X, not enable)
    GPIO.output(ENABLE_PIN_Y, not enable)

# Context manager for raw mode
@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

# Movement functions
def move_forward(motor, steps, delay=0.001):
    motor.motor_go(True, "Full", steps, delay, False, 0.05)

def move_backward(motor, steps, delay=0.001):
    motor.motor_go(False, "Full", steps, delay, False, 0.05)

class Turret:
    def __init__(self):
        print("Initializing Turret...")
        enable_motors(True)
        atexit.register(self.__turn_off_motors)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)

    def calibrate(self):
        print("Calibrate the turret...")

    def __turn_off_motors(self):
        enable_motors(False)

# Example use
if __name__ == "__main__":
    turret = Turret()
    try:
        print("Moving X axis forward...")
        move_forward(motor_x, 200)
        time.sleep(1)
        print("Moving Y axis backward...")
        move_backward(motor_y, 200)
    finally:
        GPIO.cleanup()
        print("Finished.")



