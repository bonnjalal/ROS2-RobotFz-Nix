import cv2 as cv
import numpy as np

# Load the image of the blue ball (replace with your image path)
img = cv.imread("ball_image.png")

if img is None:
    print("Error: Could not read image.")
else:
    # Convert to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Define a mask for the area of the ball (you might want to draw a small box around the ball)
    mask = np.zeros(img.shape[:2], dtype="uint8")
    cv.rectangle(mask, (50, 50), (100, 100), (255), -1)  # Example - adjust coordinates

    # Extract region of interest (the ball area)
    masked_hsv = cv.bitwise_and(hsv, hsv, mask=mask)

    # Compute the mean color from the mask region
    mean_hsv = cv.mean(masked_hsv)[0:3]

    # Print the average HSV values for the detected region
    print(
        f"The average HSV of the ball: H={mean_hsv[0]}, S={mean_hsv[1]}, V={mean_hsv[2]}"
    )

    # Display the image and mask region
    cv.imshow("Original Image", img)
    cv.imshow("Masked HSV", masked_hsv)
    cv.waitKey(0)
    cv.destroyAllWindows()
