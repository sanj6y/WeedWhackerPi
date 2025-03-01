import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

# Define GPIO pins
SERVO_PIN_1 = 12  # GPIO 18
SERVO_PIN_2 = 13  # GPIO 27

# Set frequency (50Hz for servos)
pi.set_PWM_frequency(SERVO_PIN_1, 50)
pi.set_PWM_frequency(SERVO_PIN_2, 50)

try:
    while True:
        # Move servo 1 to 0 degrees
        print("Moving servo on GPIO 12 to 0 degrees (1000 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_1, 1000)
        time.sleep(1)
        
        # Move servo 1 to 90 degrees
        print("Moving servo on GPIO 12 to 90 degrees (1500 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_1, 1500)
        time.sleep(1)
        
        # Move servo 1 to 180 degrees
        print("Moving servo on GPIO 12 to 180 degrees (2000 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_1, 2000)
        time.sleep(1)
        
        # Repeat for servo 2 or add additional logic as needed
        # Example for servo 2:
        print("Moving servo on GPIO 13 to 0 degrees (1000 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_2, 1000)
        time.sleep(1)
        
        print("Moving servo on GPIO 13 to 90 degrees (1500 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_2, 1500)
        time.sleep(1)
        
        print("Moving servo on GPIO 13 to 180 degrees (2000 µs pulse width)")
        pi.set_servo_pulsewidth(SERVO_PIN_2, 2000)
        time.sleep(1)

except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    # Cleanup
    print("Stopping all servos and cleaning up.")
    pi.set_servo_pulsewidth(SERVO_PIN_1, 0)
    pi.set_servo_pulsewidth(SERVO_PIN_2, 0)
    pi.stop()
