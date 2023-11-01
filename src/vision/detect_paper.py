import cv2
import numpy as np


def resize_frame(image): 
    scale = 40
    dim = (int(image.shape[1] * scale / 100),
            int(image.shape[0] * scale / 100))
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    return resized_image


def find_paper(image_path):

    image = cv2.imread(image_path)
    image = resize_frame(image)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3,3))
    edges = cv2.dilate(edges, kernel, iterations=2)
    cv2.imshow('edges', edges)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 600

    if contours is not None:
        for contour in contours:

            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                if cv2.contourArea(approx) > max_area:
                    cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                    break
    for point in approx:
        x, y = point[0]
        cv2.circle(image, (x, y), 5, (255, 0, 0), -1) 

    paper_corners = np.float32(approx)
    width, height = image.shape[1], image.shape[0]
    target_corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

    perspective_matrix = cv2.getPerspectiveTransform(paper_corners, target_corners)
    transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))

    cv2.imshow("Paper Detection", image)
    cv2.imshow("Transformed Image", transformed_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

find_paper("trasa3.jpg")