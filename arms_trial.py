import pigpio
import time

def smooth_move(servo, start, end, step=10, delay=0.02):
    """ Gradually moves servo from start to end. """
    if start < end:  # Moving up
        for pulse in range(start, end + 1, step):
            pi.set_servo_pulsewidth(servo, pulse)
            time.sleep(delay)
    else:  # Moving down
        for pulse in range(start, end - 1, -step):
            pi.set_servo_pulsewidth(servo, pulse)
            time.sleep(delay)


# Initialize pigpio
##def pick_up_item(): 
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

# Define GPIO pins
SERVO_PIN_1 = 12  # GPIO 12
SERVO_PIN_2 = 13  # GPIO 13

#pi.set_servo_pulsewidth(12, 1000)
#

#smooth_move(SERVO_PIN_2, 2500, 2500)
#smooth_move(SERVO_PIN_1, 1000, 1000)

smooth_move(SERVO_PIN_2, 2500, 2500)
time.sleep(1)
smooth_move(SERVO_PIN_1, 1200, 1200)
time.sleep(1)
smooth_move(SERVO_PIN_2, 800, 800)
time.sleep(1)
smooth_move(SERVO_PIN_1, 500, 500)
time.sleep(1)
smooth_move(SERVO_PIN_2, 2500, 2500)


"""
# Set frequency (50Hz for servos)
pi.set_PWM_frequency(SERVO_PIN_1, 50)
pi.set_PWM_frequency(SERVO_PIN_2, 50)

print("Opening Servo 2 (GPIO 13) to 180 degrees")
smooth_move(SERVO_PIN_2, 1000, 2000)

# Step 2: Move Servo 1 down smoothly (180 degrees)
print("Moving Servo 1 (GPIO 12) down to 180 degrees")
smooth_move(SERVO_PIN_1, 1000, 2000)

# Step 3: Close Servo 2 smoothly (0 degrees)
print("Closing Servo 2 (GPIO 13) to 0 degrees")
smooth_move(SERVO_PIN_2, 2000, 1000)

# Step 4: Move Servo 1 up smoothly (0 degrees)
print("Moving Servo 1 (GPIO 12) up to 0 degrees")
smooth_move(SERVO_PIN_1, 2000, 1000)
*/

"""