import cv2
import numpy as np 
from line import Line3D
# from detect_paper import find_paper

def resize_frame(image): 
    scale = 40
    dim = (int(image.shape[1] * scale / 100),
            int(image.shape[0] * scale / 100))
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    return resized_image


def find_paper(image_path):

    image = cv2.imread(image_path)

    is_paper = False

    gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3,3))
    edges = cv2.dilate(edges, kernel, iterations=2)
    # cv2.imshow('edges', edges)
    cv2.waitKey(0)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 600

    if contours is not None:
        for contour in contours:

            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            print(len(approx))
            if len(approx) == 4:
                print('approx 4')
                if cv2.contourArea(approx) > max_area:
                    # cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                    is_paper = True
                    paper = approx
                    
            
        if is_paper: 
<<<<<<< HEAD
            paper_corners = np.float32(paper)
=======
            paper_sorted = sort_rectangle(paper)
            print(paper_sorted)
>>>>>>> 3e36594 (sort points)
            width, height = image.shape[1], image.shape[0]
            target_corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
            perspective_matrix = cv2.getPerspectiveTransform(paper_corners, target_corners)
            transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))
            # cv2.imshow("Paper Detection", image)
            # cv2.imshow("Transformed Image", transformed_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            
    return transformed_image


def find_lines(image):

    cv2.imshow('findlines', image)
    image = cv2.GaussianBlur(image,(5,5),cv2.BORDER_DEFAULT)
    image = resize_frame(image)
    kernel = np.ones((5, 5), np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    height, width = image.shape[:2] 

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv', hsv)
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    lower_black = np.array([0, 0, 0]) 
    upper_black = np.array([180, 255, 50])

    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    cv2.imshow('black', mask_black)
    cv2.waitKey(0)

    contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    lines = []
    for contour in contours_black:
        if cv2.contourArea(contour) > 40:  # Filter out small noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.circle(image, (x,y), 2, (0,0,0), -1)
            cv2.circle(image, (x+w,y+ h), 2, (0,0,255), -1)
            line = Line3D(x, y, 1, x+w, y+h, 'black')
            line.calculate_world(width, height)
            lines.append(line)
            print("Black Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

    lines = []
    for contour in contours_red:
        if cv2.contourArea(contour) > 60:  # Filter out small noise
            x, y, w, h = cv2.boundingRect(contour)
            line = Line3D(x, y, 1, x+w, y+h, 'red')
            if line.calculate_length() > 60:
                cv2.circle(image, (x,y), 2, (0,0,255), -1)
                cv2.circle(image, (x+w,y+ h), 2, (0,0,255), -1)
                line.calculate_world(width, height)
                lines.append(line)
                print(line.__dict__)

    for contour in contours_blue:
        if cv2.contourArea(contour) > 60: 
            x, y, w, h = cv2.boundingRect(contour)
            line = Line3D(x, y, 1.5, x+w, y+h, 'blue')
            if line.calculate_length() > 60:
                cv2.circle(image, (x,y), 2, (255,0,0), -1)
                cv2.circle(image, (x+w,y+ h), 2, (255,0,0), -1)
                line.calculate_world(width, height)
                lines.append(line)
                print(line.__dict__)

    for contour in contours_green:
        if cv2.contourArea(contour) > 60:  
            line = Line3D(x, y, 1.5, x+w, y+h, 'blue')
            x, y, w, h = cv2.boundingRect(contour)
            if line.calculate_length() > 60:
                cv2.circle(image, (x,y), 2, (0, 255, 0), -1)
                cv2.circle(image, (x+w,y+ h), 2, (0, 255, 0), -1)
                line.calculate_world(width, height)
                lines.append(line)
                print(line.__dict__)

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# image = cv2.imread('trasa3.jpg')
# image = resize_frame(image.copy())
# cv2.imshow("gowno", image)

trajmap = find_paper('trasa10.jpg')
find_lines(trajmap)