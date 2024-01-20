# import the required library 
import numpy as np 
import cv2
from matplotlib import pyplot as plt 

def dist(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    order = np.argsort(dist_2)
    return nodes[order]
# read the image 
img = cv2.imread('test2.png')

# convert image to gray scale image
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# detect corners with the goodFeaturesToTrack function.
corners = cv2.goodFeaturesToTrack(gray, 150, 0.01, 15)
corners = np.intp(corners)
corners = np.resize(corners,(41,2))
h, w, _ = img.shape
print((h,w))

corners = corners[
    np.logical_and(
        np.logical_and(corners[:,1] > 60, corners[:,1] < h-100),
        np.logical_and(corners[:,0] > 60, corners[:,0] < w-100)
    )
]
corners = dist([w/2,h/2], corners)

# iterate through each corner
# making a circle at each point that we think is a corner
copy_corners = corners
trajectory = []
cock = corners[0].tolist()
print(type(cock))
trajectory.append(cock)
for i in corners:
    x, y = i.ravel()
    res = dist(i, copy_corners)[:2]
    cv2.circle(img, (x, y), 3, 255, -1)
    for p in res:
        trajectory.append(p.tolist())
        xd, yd = p.ravel()
        cv2.line(img,(x,y), (xd,yd), [255,0,0], 1)
    copy_corners = np.delete(copy_corners,0,0)
    
print(trajectory)
plt.imshow(img), plt.show()