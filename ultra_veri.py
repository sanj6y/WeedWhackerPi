import RPi.GPIO as GPIO
import time

# Define GPIO pins
TRIG = 27  # Trigger pin (T/27)
ECHO = 22  # Echo pin (E/22)

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    """Measures distance using the ultrasonic sensor."""
    GPIO.output(TRIG, False)
    time.sleep(0.1)  # Let sensor settle

    # Send 10µs pulse to trigger
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG, False)

    # Wait for echo to start
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()

    # Wait for echo to end
    stop_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    # Calculate pulse duration
    pulse_duration = stop_time - start_time

    # Convert to distance (speed of sound = 34300 cm/s)
    distance = (pulse_duration * 34300) / 2

    return round(distance, 2)

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist} cm")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nMeasurement stopped by user.")

finally:
    GPIO.cleanup()  # Clean up GPIO
