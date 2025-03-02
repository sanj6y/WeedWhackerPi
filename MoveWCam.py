import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Define motor control pins
MOTOR_LEFT_FORWARD = 17
MOTOR_LEFT_BACKWARD = 18
MOTOR_RIGHT_FORWARD = 22
MOTOR_RIGHT_BACKWARD = 23

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)

# PWM setup
pwm_left = GPIO.PWM(MOTOR_LEFT_FORWARD, 1000)  # 1kHz PWM
pwm_right = GPIO.PWM(MOTOR_RIGHT_FORWARD, 1000)
pwm_left.start(0)
pwm_right.start(0)

# OpenCV color range for detecting an orange ball
lower_orange = np.array([10, 140, 140])  # Adjust based on lighting
upper_orange = np.array([22, 255, 255])

# Open webcam (0 for built-in camera, 1 for external)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Camera opened successfully. Moving forward until the ball disappears...")

def move_forward(speed=50):
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.LOW)

def stop_motors():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Resize frame for performance
        frame = cv2.resize(frame, (600, 400))

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the orange ball
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours of the detected ball
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # If ball is detected, move forward
            print("Ball detected! Moving forward...")
            move_forward(speed=50)  # Move at 50% speed
        else:
            # If ball is not detected, stop
            print("Ball lost. Stopping.")
            stop_motors()

        # Display the mask and frame for debugging
        cv2.imshow("Ball Tracking", frame)
        cv2.imshow("Mask", mask)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")

finally:
    stop_motors()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("Cleanup completed.")
