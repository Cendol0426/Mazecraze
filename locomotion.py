import RPi.GPIO as GPIO
import RpiMotorLib
import time

StepPin1 = 17  # Motor 1 Step Pin
DirPin1 = 27   # Motor 1 Direction Pin
StepPin2 = 22  # Motor 2 Step Pin
DirPin2 = 23   # Motor 2 Direction Pin
StepPin3 = 24  # Motor 3 Step Pin
DirPin3 = 25   # Motor 3 Direction Pin
StepPin4 = 5   # Motor 4 Step Pin
DirPin4 = 6    # Motor 4 Direction Pin

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize stepper motors (use appropriate driver class and pins)
motor1 = RpiMotorLib.A4988Nema(DirPin1, StepPin1) # Front Left
motor2 = RpiMotorLib.A4988Nema(DirPin2, StepPin2) # Front Right
motor3 = RpiMotorLib.A4988Nema(DirPin3, StepPin3) # Rear Left
motor4 = RpiMotorLib.A4988Nema(DirPin4, StepPin4) # Rear Right


# Move motors (simple test)
def forward():
    # Move forward
    motor1.step(200, "full", 0.01)  # Adjust the step count and speed
    motor2.step(200, "full", 0.01)
    motor3.step(200, "full", 0.01)
    motor4.step(200, "full", 0.01)
    time.sleep(1)


def backward():
    # Move backward
    motor1.step(200, "full", -0.01)  # Adjust the step count and speed
    motor2.step(200, "full", -0.01)
    motor3.step(200, "full", -0.01)
    motor4.step(200, "full", -0.01)
    time.sleep(1)


def clockwise():
    # Rotate Clockwise
    motor1.step(200, "full", 0.01)  # Adjust the step count and speed
    motor2.step(200, "full", -0.01)
    motor3.step(200, "full", 0.01)
    motor4.step(200, "full", -0.01)
    time.sleep(1)


def anticlockwise():
    # Move backward
    motor1.step(200, "full", -0.01)  # Adjust the step count and speed
    motor2.step(200, "full", 0.01)
    motor3.step(200, "full", -0.01)
    motor4.step(200, "full", 0.01)
    time.sleep(1)