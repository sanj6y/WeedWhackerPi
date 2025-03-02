import cv2
import numpy as np
#import imutils

# Open the webcam
cap = cv2.VideoCapture(0)

# Define HSV range for RED (Two ranges due to wrap-around in HSV)
lower_red1 = np.array([0, 150, 100], dtype="uint8")
upper_red1 = np.array([10, 255, 255], dtype="uint8")
lower_red2 = np.array([170, 150, 100], dtype="uint8")
upper_red2 = np.array([180, 255, 255], dtype="uint8")

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Webcam opened successfully. Press 'q' to exit.")

while True:
    grabbed, frame = cap.read()
    if not grabbed:
        print("Error: Failed to capture frame.")
        break

    # Resize frame for better processing
    frame = imutils.resize(frame, width=600)

    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply two masks for red (red exists in two HSV ranges)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)  # Combine both masks

    # Apply Morphological Transformations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)  # Removes small noise
    mask = cv2.dilate(mask, kernel, iterations=2)  # Restores main objects

    # Find contours
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in cnts:
        area = cv2.contourArea(cnt)

        if area > 500:  # Ignore small objects
            # Fit a circle around the contour
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            perimeter = cv2.arcLength(cnt, True)
            circularity = (4 * np.pi * area) / (perimeter ** 2 + 1e-5)

            # Ball shape filtering (ensures detection is round)
            if circularity > 0.8 and 10 < radius < 100:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (255, 0, 0), -1)  # Draw center point
                print(f"Detected Ball at: (x={int(x)}, y={int(y)}, radius={int(radius)})")

    # Show frames
    cv2.imshow("Live Ball Tracking", frame)
    cv2.imshow("Mask", mask)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord("q"):
        print("Exiting...")
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
