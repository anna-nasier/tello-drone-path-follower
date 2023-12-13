import cv2
import numpy as np 
from scipy.spatial import distance as dist
# from line import Line3D
# from detect_paper import find_paper

def resize_frame(image, scale=50): 
    dim = (int(image.shape[1] * scale / 100),
            int(image.shape[0] * scale / 100))
    resized_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

    return resized_image

def gamma_correction(image, gamma=1.5):
    gamma_inv = 1.0 / gamma
    look_up_table = np.array([((i / 255.0) ** gamma_inv) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
    gamma_corrected = cv2.LUT(image, look_up_table)
    return gamma_corrected


def find_conts(img): 
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grey = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
    _, th = cv2.threshold(grey, 127, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
    cont, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return cont

def find_start(x, img):
    conts = find_conts(img)
    ret = []
    for cont in conts:
        ret.append( cv2.matchShapes(x, cont, 1, 0.0))
    idx = ret.index((max(ret)))
    beginning = conts[idx]
    # cv2.drawContours(img, conts, -1, (0,255,0), 3)
    cv2.imshow('stat', img)
    cv2.waitKey(0)

# def sort_rectangle(rect): 
#     x,y,w,h = rect
#     one = [x, y]
#     two = [x+w, y]
#     three = [x+w, y+h]
#     four = [x, y+h]
#     paper = np.float32([one, two, three, four])
#     return paper

def sort_rectangle(rect): 
    x,y,w,h = rect
    offset = 30
    one = [x+offset, y+offset]
    two = [x+w-offset, y+offset]
    three = [x+w-offset, y+h-offset]
    four = [x+ offset, y+h - offset]
    paper = np.float32([one, two, three, four])
    return paper


def find_paper(image):

    is_paper = False

    gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    kernel = np.ones((3,3))
    edges = cv2.dilate(edges, kernel, iterations=2)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 2000
    if contours is not None:
        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4:
                if cv2.contourArea(approx) > max_area:
                    is_paper = True
                    paper = cv2.boundingRect(approx)
            
        if is_paper: 
            paper_sorted = sort_rectangle(paper)
            width, height = image.shape[1], image.shape[0]
            target_corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
            perspective_matrix = cv2.getPerspectiveTransform(paper_sorted, target_corners)
            transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))            
            return transformed_image, is_paper
        else: 
            return image, is_paper
