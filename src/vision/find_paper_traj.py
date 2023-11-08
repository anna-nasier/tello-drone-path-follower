import cv2
import numpy as np 
# from detect_paper import find_paper

def resize_frame(image, scale=50): 
    dim = (int(image.shape[1] * scale / 100),
            int(image.shape[0] * scale / 100))
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    return resized_image

def sort_rectangle(rect): 
    maxim = np.amax(rect, 0)
    width, height = maxim[0]
    left = []
    top = []
    for point in rect: 
        x,y = point[0]
        if y < height/2 and x < width/2 :
            one = point
        if y < height/2 and x > width/2:
            two = point
        if y > height/2 and x > width/2:
            three = point
        if y > height/2 and x < width/2:
            four = point
    paper = np.float32([one, two, three, four])
    # paper = ([one, two, three, four])
    # print(paper)
    return paper

def find_paper(image_path):

    image = cv2.imread(image_path)
    image = resize_frame(image)

    is_paper = False

    gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3,3))
    edges = cv2.dilate(edges, kernel, iterations=2)
    cv2.imshow('edges', edges)
    cv2.waitKey(0)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 2000

    if contours is not None:
        for contour in contours:

            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4:
                if cv2.contourArea(approx) > max_area:
                    is_paper = True
                    paper = approx
                    # print(approx, type(approx))
                    
            
        if is_paper: 
            paper_sorted = sort_rectangle(paper)
            # print(paper_sorted)
            # print(paper_sorted[0][0][0], paper_sorted[2][0][0], paper_sorted[0][0][1], paper_sorted[2][0][1])
            # roi = image[paper_sorted[0][0][1]:paper_sorted[2][0][1], paper_sorted[0][0][0]:paper_sorted[2][0][0]]
            # cv2.imshow('roi', roi)
            # cv2.waitKey(0)
            print(paper_sorted)
            # cv2.drawContours(image, [paper], -1, (0, 255, 0), 2)
            width, height = image.shape[1], image.shape[0]
            target_corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
            perspective_matrix = cv2.getPerspectiveTransform(paper_sorted, target_corners)
            transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))
            cv2.imshow("Paper Detection", image)
            cv2.imshow("Transformed Image", transformed_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            return transformed_image
            # return roi
        else: 
            return image


def find_lines(image):

    cv2.imshow('findlines', image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv', hsv)


    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])

    lower_blue = np.array([100, 40, 40])
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
            # cv2.drawContours(image, contour, -1, (0,0,255), 2)
            print("Red Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

    for contour in contours_blue:
        if cv2.contourArea(contour) > 40: 
            x, y, w, h = cv2.boundingRect(contour)
            cv2.circle(image, (x,y), 2, (255,0,0), -1)
            cv2.circle(image, (x+w,y+ h), 2, (255,0,0), -1)
            # cv2.drawContours(image, contour, -1, (255,0,0), 2)
            print("Blue Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

    for contour in contours_green:
        if cv2.contourArea(contour) > 40:  
            x, y, w, h = cv2.boundingRect(contour)
            cv2.circle(image, (x,y), 2, (0, 255, 0), -1)
            cv2.circle(image, (x+w,y+ h), 2, (0, 255, 0), -1)
            # cv2.drawContours(image, contour, -1, (0, 255, 0), 2)
            print("Green Line: x={}, y={}, w={}, h={}".format(x, y, w, h))

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# image = cv2.imread('trasa3.jpg')
# image = resize_frame(image.copy())
# cv2.imshow("gowno", image)

trajmap = find_paper('trasa6.jpg')
find_lines(trajmap)