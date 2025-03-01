import RPi.GPIO as GPIO
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

def move_forward(speed):
    # Both motors forward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def move_backward(speed):
    # Both motors backward
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_left(speed):
    # Left motor backward, Right motor forward
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def turn_right(speed):
    # Left motor forward, Right motor backward
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm1.ChangeDutyCycle(speed)
    pwm2.ChangeDutyCycle(speed)

def stop_motors():
    # Stop both motors
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)

try:
    while True:
        command = input("Enter command (w: forward, s: backward, a: left, d: right, x: stop, q: quit): ").strip().lower()
        if command == 'a':
            print("Moving forward...")
            move_forward(speed=50)  # 50% speed
        elif command == 'd':
            print("Moving backward...")
            move_backward(speed=50)  # 50% speed
        elif command == 'w':
            print("Turning left...")
            turn_left(speed=50)  # 50% speed
        elif command == 's':
            print("Turning right...")
            turn_right(speed=50)  # 50% speed
        elif command == 'x':
            print("Stopping...")
            stop_motors()
        elif command == 'q':
            print("Exiting program.")
            break
        else:
            print("Invalid command. Please enter 'w', 's', 'a', 'd', 'x', or 'q'.")

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    stop_motors()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("GPIO cleanup completed.")
