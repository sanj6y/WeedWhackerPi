import RPi.GPIO as GPIO
import cv2
import numpy as np
import imutils
import time

duration = 2
# GPIO pin assignments for Motor 1 (left side)
IN1 = 23  # GPIO pin for Motor 1 direction
IN2 = 24  # GPIO pin for Motor 1 direction
ENA = 25  # GPIO pin for Motor 1 enable (PWM)

# GPIO pin assignments for Motor 2 (right side)
IN3 = 5   # GPIO pin for Motor 2 direction
IN4 = 6   # GPIO pin for Motor 2 direction
ENB = 26  # GPIO pin for Motor 2 enable (PWM)


SERVO1 = 13  # Servo for Up/Down movement
SERVO2 = 12  # Servo for Collect/Not Collect



# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)

GPIO.setup(SERVO1, GPIO.OUT)
GPIO.setup(SERVO2, GPIO.OUT)


# Initialize PWM on ENA and ENB with 100Hz frequency
pwm1 = GPIO.PWM(ENA, 100)
pwm2 = GPIO.PWM(ENB, 100)
pwm1.start(0)
pwm2.start(0)

# Initialize PWM for Servos (50Hz frequency)
servo1 = GPIO.PWM(SERVO1, 50)  # Servo 1 (Up/Down)
servo2 = GPIO.PWM(SERVO2, 50)  # Servo 2 (Collect/Not Collect)
# servo1.start(7.5)  # Neutral position
# servo2.start(7.5)  # Neutral position

# Open webcam for tracking
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)


# Define HSV range for red ball
lower_red1 = np.array([0, 150, 100], dtype="uint8")
upper_red1 = np.array([10, 255, 255], dtype="uint8")
lower_red2 = np.array([170, 150, 100], dtype="uint8")
upper_red2 = np.array([180, 255, 255], dtype="uint8")

# Motor movement functions
def move_forward(speed=50, duration=2):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motors()

def move_backward(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motors()

def turn_left(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motors()
    time.sleep(duration)
    stop_motors()

def turn_right(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)
    time.sleep(duration)
    stop_motors()

def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)


# Servo control functions
def servo1_up():
    """ Move Servo 1 (Up) """
    print("Servo 1 Moving UP")
    servo1.ChangeDutyCycle(12.5)  # Adjust angle

def servo1_down():
    """ Move Servo 1 (Down) """
    print("Servo 1 Moving DOWN")
    servo1.ChangeDutyCycle(2.5)  # Adjust angle

def servo2_collect():
    """ Activate Collect Mechanism """
    print("Servo 2 Collecting")
    servo2.ChangeDutyCycle(12.5)  # Adjust angle

def servo2_release():
    """ Deactivate Collect Mechanism """
    print("Servo 2 Releasing")
    servo2.ChangeDutyCycle(2.5)  # Adjust angle


def detect_ball():
    """ Detects a red ball using OpenCV and saves an image if detected """
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot capture frame.")
        return False

    frame = imutils.resize(frame, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for red ball
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Find contours
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(cnts) > 0:
        cnt = max(cnts, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(cnt)

        if radius > 50:  # Only track large enough objects
            print("Ball Detected! Stopping and capturing image.")

            # Draw circle around the detected ball
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 3)

            # Save the image
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"ball_detected_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Image saved: {filename}")

            # Stop the robot
            stop_motors()
            return True

    return False

def search_and_follow_ball():
    """ Rotates in steps, moves forward if ball detected, and captures an image """
    try:
        while True:
            ball_found = False  # Reset detection flag

            # Rotate 360° in 4 quarters (90° each)
            for _ in range(6):
                print("Turning to search for ball...")
                time.sleep(2)  # Simulating slow movement
                move_forward(0.35,0.35)

                # Check if ball is detected
                if detect_ball():
                    print("Ball Detected! Moving forward.")
                    time.sleep(2)  # Move for 2 seconds
                    stop_motors()  # Stop after moving
                    ball_found = True
                    break  # Stop rotating and move towards the ball

            if not ball_found:
                print("No ball detected. Restarting search...")

    except KeyboardInterrupt:
        print("Exiting ball tracking mode.")
    finally:
        stop_motors()




# Main control loop
try:
    while True:
        command = input("Enter command (w: forward, s: backward, a: left, d: right, x: stop, e: follow ball, q: quit): ").strip().lower()
        if command == 'w':
            print("Moving forward...")
            turn_right(speed=20)
        elif command == 's':
            print("Moving backward...")
            turn_left(speed=20)
        elif command == 'a':
            print("Turning left...")
            move_backward(speed=20)
        elif command == 'd':
            print("Turning right...")
            move_forward(speed=20)
        elif command == 'x':
            print("Stopping...")
            stop_motors()
        elif command == 'u':  # Servo 1 Up
            servo1_up()
        elif command == 'l':  # Servo 1 Down
            servo1_down()
        elif command == 'c':  # Servo 2 Collect
            servo2_collect()
        elif command == 'r':  # Servo 2 Release
            servo2_release()
        elif command == 'q':  # Quit
            print("Exiting program.")
            break
        elif command == 'e':  # Quit
            print("atoumus  mode.")
            search_and_follow_ball()
        else:
            print("Invalid command.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    stop_motors()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("GPIO cleanup completed.")
