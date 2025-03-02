import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Define motor control pins
MOTOR_LEFT_FORWARD = 17
MOTOR_LEFT_BACKWARD = 18
MOTOR_RIGHT_FORWARD = 22
MOTOR_RIGHT_BACKWARD = 23
ENA = 25  # Left motor enable (PWM)
ENB = 26  # Right motor enable (PWM)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LEFT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_LEFT_BACKWARD, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_FORWARD, GPIO.OUT)
GPIO.setup(MOTOR_RIGHT_BACKWARD, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# PWM setup for speed control
pwm_left = GPIO.PWM(ENA, 1000)  # 1kHz PWM
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

# OpenCV HSV range for RED detection
lower_red1 = np.array([0, 150, 100], dtype="uint8")
upper_red1 = np.array([10, 255, 255], dtype="uint8")
lower_red2 = np.array([170, 150, 100], dtype="uint8")
upper_red2 = np.array([180, 255, 255], dtype="uint8")

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Searching for the ball...")

def turn_robot(speed=30):
    """Turns the robot incrementally"""
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.HIGH)  # One wheel forward, one backward
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(0.5)  # Incremental turn delay

def stop_robot():
    """Stops the robot"""
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

try:
    while True:
        grabbed, frame = cap.read()
        if not grabbed:
            print("Error: Failed to capture frame.")
            break

        # Resize frame for faster processing
        frame = cv2.resize(frame, (600, 400))

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply two masks for red (handles HSV wrap-around)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Remove small noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_detected = False

        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area > 500:  # Ignore small objects
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                perimeter = cv2.arcLength(cnt, True)
                circularity = (4 * np.pi * area) / (perimeter ** 2 + 1e-5)

                # Ensure it's a round object within the expected size
                if circularity > 0.8 and 10 < radius < 100:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # Draw ball outline
                    cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)  # Draw center
                    print(f"Ball detected at: (x={int(x)}, y={int(y)}, radius={int(radius)})")
                    ball_detected = True
                    break  # Stop checking once a ball is found

        if ball_detected:
            print("Ball found! Stopping...")
            stop_robot()
        else:
            print("Ball not found. Turning...")
            turn_robot(speed=30)  # Adjust speed as needed

        # **Show camera feed**
        cv2.imshow("Live Camera Feed", frame)  # Shows what the camera sees
        cv2.imshow("Mask (Ball Detection)", mask)  # Shows ball tracking mask

        # Exit on 'q' key
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Exiting...")
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    stop_robot()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("Cleanup completed.")
