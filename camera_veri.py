import cv2

# Open video capture (0 for default camera, try 1 if 0 doesn't work)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Camera not detected.")
else:
    print("Camera is working. Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        
        cv2.imshow("Live Camera Feed", frame)

        # Press 'q' to exit the video window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
