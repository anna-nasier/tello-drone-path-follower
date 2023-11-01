import cv2
import numpy as np 


def resize_frame(image): 
    scale = 70
    dim = (int(image.shape[1] * scale / 100),
            int(image.shape[0] * scale / 100))
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    return resized_image

image = cv2.imread('trasa1.jpg')
image = resize_frame(image)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


lower_red = np.array([0, 50, 50])
upper_red = np.array([10, 255, 255])

lower_blue = np.array([100, 50, 50])
upper_blue = np.array([130, 255, 255])

lower_green = np.array([40, 40, 40])
upper_green = np.array([80, 255, 255])


mask_red = cv2.inRange(hsv, lower_red, upper_red)
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
mask_green = cv2.inRange(hsv, lower_green, upper_green)


contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Extract and print the coordinates of the lines
for contour in contours_red:
    if cv2.contourArea(contour) > 40:  # Filter out small noise
        x, y, w, h = cv2.boundingRect(contour)
        cv2.circle(image, (x,y), 2, (0,0,255), -1)
        cv2.circle(image, (x+w,y+ h), 2, (0,0,255), -1)
        print("Red Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

for contour in contours_blue:
    if cv2.contourArea(contour) > 40:  # Filter out small noise
        x, y, w, h = cv2.boundingRect(contour)
        cv2.circle(image, (x,y), 2, (255,0,0), -1)
        cv2.circle(image, (x+w,y+ h), 2, (255,0,0), -1)
        print("Blue Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

for contour in contours_green:
    if cv2.contourArea(contour) > 40:  # Filter out small noise
        x, y, w, h = cv2.boundingRect(contour)
        cv2.circle(image, (x,y), 2, (0, 255, 0), -1)
        cv2.circle(image, (x+w,y+ h), 2, (0, 255, 0), -1)
        print("Green Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()