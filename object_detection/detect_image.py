import cv2
import numpy as np

# run the following terminal commands to install the ssd model locally
# i think it should be installed on the raspberry pi but just in case
# mkdir -p ~/object_detection && cd ~/object_detection
# wget -O ssd_mobilenet_v3.pb https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel
# wget -O ssd_mobilenet_v3.prototxt https://github.com/chuanqi305/MobileNet-SSD/raw/master/deploy.prototxt


# Load the pre-trained MobileNet SSD model
prototxt_path = "ssd_mobilenet_v3.prototxt"
model_path = "ssd_mobilenet_v3.pb"
net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

# Load class labels (COCO dataset)
CLASSES = ["background", "airplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "dining table",
           "dog", "horse", "motorbike", "person", "potted plant", "sheep",
           "sofa", "train", "tv", "apple", "orange", "banana", "ball"]  # Add relevant objects

# Read the test image
image = cv2.imread("image.jpeg")
(h, w) = image.shape[:2]

# Convert image into a blob (preprocess for MobileNet)
blob = cv2.dnn.blobFromImage(image, 0.007843, (300, 300), 127.5)
net.setInput(blob)

# Run forward pass for object detection
detections = net.forward()

# Process detections
for i in range(detections.shape[2]):
    confidence = detections[0, 0, i, 2]  # Confidence score
    if confidence > 0.5:  # Filter low-confidence detections
        class_id = int(detections[0, 0, i, 1])  # Object class ID
        label = CLASSES[class_id] if class_id < len(CLASSES) else "Unknown"

        # Get bounding box
        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
        (startX, startY, endX, endY) = box.astype("int")

        # Draw bounding box and label
        cv2.rectangle(image, (startX, startY), (endX, endY), (0, 255, 0), 2)
        text = f"{label}: {confidence:.2f}"
        cv2.putText(image, text, (startX, startY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Save the output image
cv2.imwrite("classified_output.jpeg", image)

print("Detection complete! Check classified_output.jpeg")
