import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Define motor control pins
MOTOR_LEFT_FORWARD = 23
MOTOR_LEFT_BACKWARD = 24
MOTOR_RIGHT_FORWARD = 5
MOTOR_RIGHT_BACKWARD = 6
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

def move_forward(speed=10, duration=0.3):
    """Turns the robot for a short duration, then stops"""
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.HIGH)  # One wheel forward, one backward
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    time.sleep(duration)  # Turn for 0.3 seconds
    stop_robot()  # Stop after turning

def turn_robot(speed=10, duration=0.3):
    """Moves the robot forward in increments, then stops"""
    print(f"Moving forward for {duration} seconds...")
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.HIGH)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.LOW)

    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)
    
    time.sleep(duration)  # Move forward for a short time
    stop_robot()  # Stop after each movement

def stop_robot():
    """Stops the robot"""
    print("Stopping motors...")
    GPIO.output(MOTOR_LEFT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_FORWARD, GPIO.LOW)
    GPIO.output(MOTOR_RIGHT_BACKWARD, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    print("Robot stopped.")

ball_was_detected = False  # Track if we found the ball at least once

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
                    ball_was_detected = True  # Mark that we found the ball
                    break  # Stop checking once a ball is found

        if ball_was_detected:
            if ball_detected:
                print("Ball found! Moving forward in increments...")
                move_forward(speed=20, duration=0.3)  # Move forward for 0.3 seconds
                
            else:
                print("Ball lost. Stopping permanently.")
                stop_robot()
                break  # **Exit the loop permanently after stopping**

        else:
            print("Ball not found. Turning...")
            turn_robot(speed=5, duration=0.3)  # **Turn for 0.3 seconds, then stop**
            time.sleep(1)  # Small delay to allow camera processing

        # **Show camera feed**
        cv2.imshow("Live Camera Feed", frame)  # Shows what the camera sees
        cv2.imshow("Mask (Ball Detection)", mask)  # Shows ball tracking mask

        # Exit on 'q' key
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
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
