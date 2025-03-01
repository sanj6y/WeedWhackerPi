import cv2

# 1. Read an image (replace 'image.jpeg' with your actual file path)
img = cv2.imread('image.jpeg')

if img is None:
    print("Image not found or unable to open")
else:
    # 2. Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 3. Save the grayscale image
    cv2.imwrite('gray_image.jpg', gray)

    print("Image converted to grayscale and saved as gray_image.jpg!")
