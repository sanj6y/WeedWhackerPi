import RPi.GPIO as GPIO
import cv2
import numpy as np
import imutils
import time

# GPIO pin assignments for Motor 1 (left side)
IN1 = 23  # GPIO pin for Motor 1 direction
IN2 = 24  # GPIO pin for Motor 1 direction
ENA = 25  # GPIO pin for Motor 1 enable (PWM)

# GPIO pin assignments for Motor 2 (right side)
IN3 = 5   # GPIO pin for Motor 2 direction
IN4 = 6   # GPIO pin for Motor 2 direction
ENB = 26  # GPIO pin for Motor 2 enable (PWM)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)

# Initialize PWM on ENA and ENB with 100Hz frequency
pwm1 = GPIO.PWM(ENA, 100)
pwm2 = GPIO.PWM(ENB, 100)
pwm1.start(0)
pwm2.start(0)

# Open webcam for tracking
cap = cv2.VideoCapture(0)

# Define HSV range for red ball
lower_red1 = np.array([0, 150, 100], dtype="uint8")
upper_red1 = np.array([10, 255, 255], dtype="uint8")
lower_red2 = np.array([170, 150, 100], dtype="uint8")
upper_red2 = np.array([180, 255, 255], dtype="uint8")

# Motor movement functions
def move_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def move_backward(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_left(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_right(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def stop_motors():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

def find_ball_and_follow():
    """
    Uses OpenCV to detect a red ball and move the robot towards it.
    """
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot capture frame.")
                break

            frame = imutils.resize(frame, width=600)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create masks for red ball
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            # Find contours
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts) > 0:
                # Get the largest contour
                cnt = max(cnts, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(cnt)

                if radius > 10:  # Only track objects that are large enough
                    center_x = int(x)
                    frame_center = frame.shape[1] // 2  # Get center of the frame

                    if center_x < frame_center - 50:  # Ball is on the left
                        print("Ball detected on LEFT → Turning Left")
                        move_forward(speed=15)
                    elif center_x > frame_center + 50:  # Ball is on the right
                        print("Ball detected on RIGHT → Turning Right")
                        move_backward(speed=15)
                    else:  # Ball is centered
                        print("Ball centered → Moving Forward")
                        turn_right(speed=15)

            else:
                print("No ball detected → Stopping")
                stop_motors()

            # Display video feed
            cv2.imshow("Ball Tracking", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Exiting ball tracking mode.")
                break

    except KeyboardInterrupt:
        print("Exiting ball tracking mode.")

    finally:
        stop_motors()
        cap.release()
        cv2.destroyAllWindows()

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
        elif command == 'e':
            print("Starting ball following mode...")
            find_ball_and_follow()()
        elif command == 'q':
            print("Exiting program.")
            break
        else:
            print("Invalid command. Please enter 'w', 's', 'a', 'd', 'x', 'e', or 'q'.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    stop_motors()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("GPIO cleanup completed.")
