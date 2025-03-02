import cv2

cap = cv2.VideoCapture(1)  # Try 1 if 0 doesn't work

if not cap.isOpened():
    print("Error: Camera not detected.")
else:
    print("Camera is working.")
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Test Frame", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

cap.release()
