from pynput import keyboard
import pigpio
import time

pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon.")
    exit()

SERVO_PIN_2 = 13  # GPIO 13
UP_POSITION = 2000
DOWN_POSITION = 1000
NEUTRAL_POSITION = 1500

pi.set_servo_pulsewidth(SERVO_PIN_2, NEUTRAL_POSITION)

def on_press(key):
    try:
        if key.char == '1':  # Move up
            print("Moving UP...")
            pi.set_servo_pulsewidth(SERVO_PIN_2, UP_POSITION)
        elif key.char == '2':  # Move down
            print("Moving DOWN...")
            pi.set_servo_pulsewidth(SERVO_PIN_2, DOWN_POSITION)
    except AttributeError:
        pass

def on_release(key):
    pi.set_servo_pulsewidth(SERVO_PIN_2, NEUTRAL_POSITION)  # Reset when key is released
    if key == keyboard.Key.esc:  # Exit on Esc key
        return False

print("Press '1' to move UP, '2' to move DOWN. Release to stop.")
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

pi.set_servo_pulsewidth(SERVO_PIN_2, 0)  # Turn off servo
pi.stop()
