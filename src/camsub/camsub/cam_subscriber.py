import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from camsub.find_paper_traj import find_paper
from camsub.line import Line3D
from camsub.line import find_lines
import math
from numpy import ones,vstack
from numpy.linalg import lstsq
from sklearn.neighbors import NearestNeighbors
import networkx as nx
import matplotlib.pyplot as plt

# def dist_from_center(img, line):
#    height, width, _ = img.shape
#    center = [width/2, height/2] 
#    cv2.rectangle(img, (int(width*0.1), int(height*0.1)), (int(width*0.8), int(height*0.8)), (255, 0, 0), 3, cv2.LINE_AA)
#    pt1 = [line[0], line[1]]
#    pt2 = [line[2], line[3]]
#    dist_x = width - center[0]
#    dist_y = height - center[1]
#    dist_xy = math.sqrt((center[0] - width)**2 + (center[1]-height)**2)
#    dist_pt1 = math.sqrt((pt1[0]- center[0])**2 + (pt1[1] - center[1])**2)
#    dist_pt2 = math.sqrt((pt2[0]- center[0])**2 + (pt2[1] - center[1])**2)
#    if dist_pt1/dist_xy > 0.7 or dist_pt2/dist_xy > 0.7:
#       return False
#    elif abs(pt1[0]-center[0])/dist_x > 0.8 or abs(pt1[1]-center[1]) / dist_y > 0.8:
#      return False
#    elif abs(pt2[0]-center[0])/dist_x > 0.8 or abs(pt2[1] - center[1]) / dist_y > 0.8:
#      return False
#    else:
#       return True
  
def dist_from_center(img, point):
   height, width, _ = img.shape
   center = [width/2, height/2] 
   cv2.rectangle(img, (int(width*0.1), int(height*0.1)), (int(width*0.8), int(height*0.8)), (255, 0, 0), 3, cv2.LINE_AA)
   pt1 = point[0]
   dist_x = width - center[0]
   dist_y = height - center[1]
   dist_xy = math.sqrt((center[0] - width)**2 + (center[1]-height)**2)
   dist_pt1 = math.sqrt((pt1[0]- center[0])**2 + (pt1[1] - center[1])**2)
   if dist_pt1/dist_xy > 0.7:
      return False
   elif abs(pt1[0]-center[0])/dist_x > 0.8 or abs(pt1[1]-center[1]) / dist_y > 0.8:
     return False
   else:
      return True
   

def get_line_eq(line):
  points = [(line[0],line[1]),(line[2],line[3])]
  x_coords, y_coords = zip(*points)
  A = vstack([x_coords,ones(len(x_coords))]).T
  m, c = lstsq(A, y_coords)[0]
  return m, c


def remove_multilines(line1, line2):
  remove = True
  a1 = get_line_eq(line1)
  a2 = get_line_eq(line2)
  if a1 < 0 and a2 > 0: 
    remove = False
  if a1 > 0 and a2 < 0:
    remove = False
  if a1 < 0 and a2 < 0: 
    if line2[0] > line1[2] or line2[2] > line1[0]:
      remove = False
  if a1 > 0 and a2 > 0: 
    if line1[2] < line2[0] or line2[2] < line1[0]:
      remove = False
  return remove

def is_aligned(line1, line2): 
  a1, b1 = get_line_eq(line1)
  a2, b2 = get_line_eq(line2)
  line = False
  if a1 < 0 and a2 < 0: 
    if abs(a1 - a2) < 5: 
      if abs(b1 - b2) < 20: 
        line = True
  return line
  
def dist(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    order = np.argsort(dist_2)
    return nodes[order]

class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(
      Image, 
      'image_raw', 
      self.listener_callback, 
      10)
    self.subscription
    self.br = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    cf = self.br.imgmsg_to_cv2(data)
    cf_cut, is_paper = find_paper(cf)
    self.get_logger().info(f'{cf_cut.shape[0], cf_cut.shape[1]}')
    if is_paper:
      grey = cv2.cvtColor(cf_cut.copy(), cv2.COLOR_BGR2GRAY)
      corners = cv2.goodFeaturesToTrack(grey, 60, 0.008, 10)
      self.get_logger().info(f'{corners.shape}')
      if corners is not None:
        corners = corners.astype(int)
        corners = [c for c in corners if dist_from_center(cf_cut, c)]
        if len(corners) > 1:
          corners = np.intp(corners)
          corners = np.resize(corners,(41,2))
          h, w, _ = cf_cut.shape
          corners = dist([w/2,h/2], corners)
          copy_corners = corners
          trajectory = []
          print(corners)
          cock = corners[0].tolist()

          trajectory.append(cock)
          for i in corners:
              x, y = i.ravel()
              res = dist(i, copy_corners)[:2]
              cv2.circle(cf_cut, (x, y), 3, 255, -1)
              for p in res:
                  trajectory.append(p.tolist())
                  xd, yd = p.ravel()
                  cv2.line(cf_cut,(x,y), (xd,yd), [255,0,0], 1)
              copy_corners = np.delete(copy_corners,0,0)
          self.get_logger().info(f'{trajectory}')
        cv2.imshow('lines', cf_cut)


    ## nearest neigbors
    ''' 
      self.get_logger().info(f'{corners}')
      q = 0
      points = []
      for c in corners: 
        q = q+1
        x, y = c.ravel()
        if dist_from_center(cf_cut, c):
          points.append(c)
          cv2.putText(cf_cut, str(q), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0,0,255), 2, cv2.LINE_AA)
      cor = corners.flatten()
      cor = np.reshape(cor, (-1, 2))
      clf = NearestNeighbors(n_neighbors=2)
      clf.fit(cor)
      G = clf.kneighbors_graph()
      T = nx.from_scipy_sparse_array(G)
      order = list(nx.dfs_preorder_nodes(T, 0))
      self.get_logger().info(f'G: {G} \n T: {T}')
      xx = cor[order][0]
      yy = cor[order][1]
      self.get_logger().info(f'{xx}, {yy}, {cor}')
      # plt.plot(xx, yy)
      # plt.show()
      cv2.imshow('x', cf_cut)
      '''

      ## FITLINE 
      
    '''
      # grey = cv2.GaussianBlur(grey,(5,5),cv2.BORDER_DEFAULT)
      # grey = cv2.bilateralFilter(grey, 15, 20, 100) 
      # _, th1 = cv2.threshold(grey,127,255,cv2.THRESH_BINARY)
      # th2 = cv2.adaptiveThreshold(grey,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
      th3 = cv2.adaptiveThreshold(grey,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
      contours, hier = cv2.findContours(th3,cv2.RETR_TREE,cv2.CHAIN_APPROX_TC89_KCOS)
      # merge_conts(contours)
      self.get_logger().info(f"{contours}")
      # self.get_logger().info(f"{hier}")
      grey = cv2.drawContours(grey.copy(), contours, -1, (128, 128, 128), 5)
      # for cont, h in zip(contours, hier):
      # self.get_logger().info(f"{hier}")
      for i,cnt in enumerate(contours):
        [vx, vy, x, y] = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((grey.shape[1]-x)*vy/vx)+y)
        if len(cnt) > 5:
          try:
              cv2.line(cf_cut,(grey.shape[1]-1,righty),(0,lefty),255,2)
          except: 
            pass
        # # self.get_logger().info(type(th1))
        # images = [grey, th1, th2, th3]
      cv2.imshow('lines', grey)
      cv2.imshow('x', cf_cut)
      '''

      
   ## HOUGH LINES
      
    '''
      canny = cv2.Canny(grey, 50, 200, None, 3)
      linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, 50, None, 50, 10)
      self.get_logger().info(f"{linesP}")
      if linesP is not None:
          for lxd in range(0, len(linesP)):
              lll = linesP[lxd][0]
              if dist_from_center(cf_cut, lll):
                cv2.rectangle(cf_cut, (lll[0], lll[1]), (lll[2], lll[3]), (0,255,255), 3, cv2.LINE_AA)
          lines = [linesP[0][0]]
          for j in range (1, len(linesP)): 
              l = linesP[j][0]
              ll = linesP[j-1][0]
              if not remove_multilines(l, ll):
                lines.append(ll)
          for l in lines:
              if dist_from_center(cf_cut, l):
                cv2.rectangle(cf_cut, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

      cv2.imshow('lines', cf_cut)
      # plt.show()
    '''
      
    ## CORNERS 
    
    cv2.imshow("camera", cf)
    cv2.waitKey(1)
  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()