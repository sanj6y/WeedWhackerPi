
# Open video capture (0 for default camera, try 1 if 0 doesn't work)
import cv2
import numpy as np

# run the following terminal commands to install the ssd model locally
# i think it should be installed on the raspberry pi but just in case
# mkdir -p ~/object_detection && cd ~/object_detection
# wget -O ssd_mobilenet_v3.pb https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel
# wget -O ssd_mobilenet_v3.prototxt https://github.com/chuanqi305/MobileNet-SSD/raw/master/deploy.prototxt

# Load pre-trained MobileNet SSD model
prototxt_path = "ssd_mobilenet_v3.prototxt"
model_path = "ssd_mobilenet_v3.pb"
net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

# Define object classes
CLASSES = ["background", "airplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "dining table",
           "dog", "horse", "motorbike", "person", "potted plant", "sheep",
           "sofa", "train", "tv", "apple", "orange", "banana", "ball"]

# Open webcam
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

    # Get frame dimensions
    (h, w) = frame.shape[:2]

    # Convert to blob and pass through the network
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:  # Confidence threshold
            class_id = int(detections[0, 0, i, 1])
            label = CLASSES[class_id] if class_id < len(CLASSES) else "Unknown"

            # Get bounding box
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # Draw bounding box if it's a ball
            if label == "ball":
                cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.putText(frame, f"Ball: {confidence:.2f}", (startX, startY - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Display output
    cv2.imshow("Live Object Detection", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()