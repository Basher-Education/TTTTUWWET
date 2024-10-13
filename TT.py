import time
import threading
import atexit
import sys
import termios
import contextlib
import os
from DRV8825 import DRV8825  # Import the correct driver for WAVESHARE

# Import necessary libraries, with debug prints to track each step
try:
    import cv2
    print("OpenCV imported successfully.")
except Exception as e:
    print("Warning: OpenCV not installed:", e)
    print("To use motion detection, make sure you've properly configured OpenCV.")

try:
    import imutils
    print("imutils imported successfully.")
except ImportError:
    print("Installing imutils library...")
    os.system("pip3 install imutils --user")

try:
    import RPi.GPIO as GPIO
    print("GPIO library imported.")
except ImportError as e:
    print("Warning: RPi.GPIO library not found:", e)

# Set up GPIO cleanup at exit
atexit.register(GPIO.cleanup)

# Define constants for user parameters
MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False
MAX_STEPS_X = 30
MAX_STEPS_Y = 15
RELAY_PIN = 22

# Define motor GPIO pins
DIR_PIN_X = 20  # Replace with your actual GPIO pin numbers
STEP_PIN_X = 21
ENABLE_PIN_X = 16

DIR_PIN_Y = 19
STEP_PIN_Y = 26
ENABLE_PIN_Y = 13

# Initialize the motors using the DRV8825 class
motor_x = DRV8825(dir_pin=DIR_PIN_X, step_pin=STEP_PIN_X, enable_pin=ENABLE_PIN_X)
motor_y = DRV8825(dir_pin=DIR_PIN_Y, step_pin=STEP_PIN_Y, enable_pin=ENABLE_PIN_Y)

# Enable motors
motor_x.set_enable(True)
motor_y.set_enable(True)

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

# Movement methods
def move_forward(motor, steps, delay=0.001):
    motor.set_direction(True)
    for _ in range(steps):
        motor.step()
        time.sleep(delay)

def move_backward(motor, steps, delay=0.001):
    motor.set_direction(False)
    for _ in range(steps):
        motor.step()
        time.sleep(delay)

class VideoUtils:
    @staticmethod
    def live_video(camera_port=0):
        print("Starting live video...")
        video_capture = cv2.VideoCapture(camera_port)
        while True:
            ret, frame = video_capture.read()
            if not ret:
                print("Failed to capture frame.")
                break
            cv2.imshow('Video', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        video_capture.release()
        cv2.destroyAllWindows()
        print("Closed live video.")

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)
        firstFrame = None
        tempFrame = None
        count = 0
        while True:
            grabbed, frame = camera.read()
            if not grabbed:
                break
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\n Waiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, _ = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt

class Turret:
    def __init__(self, friendly_mode=True):
        print("Initializing Turret...")
        self.friendly_mode = friendly_mode
        self.current_x_steps = 0
        self.current_y_steps = 0

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(RELAY_PIN, GPIO.OUT)
            GPIO.output(RELAY_PIN, GPIO.LOW)
            print("GPIO initialized for relay on pin", RELAY_PIN)
        except Exception as e:
            print("Error initializing GPIO:", e)

    def calibrate(self):
        print("Please calibrate the tilt of the gun so that it is level. Commands: (w) moves up, (s) moves down. Press (enter) to finish.\n")
        self.__calibrate_y_axis()
        print("Please calibrate the yaw of the gun so that it aligns with the camera. Commands: (a) moves left, (d) moves right. Press (enter) to finish.\n")
        self.__calibrate_x_axis()
        print("Calibration finished.")

    def __calibrate_x_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break
                    elif ch == "a":
                        move_backward(motor_x, 5)
                    elif ch == "d":
                        move_forward(motor_x, 5)
                    elif ch == "\n":
                        break
            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __calibrate_y_axis(self):
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break
                    elif ch == "w":
                        move_forward(motor_y, 5)
                    elif ch == "s":
                        move_backward(motor_y, 5)
                    elif ch == "\n":
                        break
            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __turn_off_motors(self):
        motor_x.set_enable(False)
        motor_y.set_enable(False)




