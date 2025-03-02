import cv2
import numpy as np
import imutils

# Define a strict HSV color range for orange
lower_orange = np.array([10, 140, 140])  # Adjusted for better filtering
upper_orange = np.array([22, 255, 255])

# Open webcam (0 for built-in camera, 1 for external camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Camera opened successfully.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Resize for better performance
    frame = imutils.resize(frame, width=600)

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for orange
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Apply Morphological Transformations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Removes small noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Closes small holes

    # Apply Gaussian blur to smooth edges
    blurred = cv2.GaussianBlur(mask, (9, 9), 0)

    # Detect circles using Hough Transform
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.5, minDist=30,
                               param1=70, param2=30, minRadius=10, maxRadius=100)

    # Draw circles if found
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            x, y, r = i[0], i[1], i[2]
            cv2.circle(frame, (x, y), r, (0, 255, 0), 3)  # Draw the circle
            cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)  # Draw the center

            print(f"Detected Orange Ball at: (x={x}, y={y}, radius={r})")

    # Show the processed frames
    cv2.imshow("Orange Ball Tracking", frame)
    cv2.imshow("Mask", mask)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
