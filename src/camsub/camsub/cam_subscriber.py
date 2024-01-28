import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from camsub.find_paper_traj import find_paper, gamma_correction
import math
from geometry_msgs.msg import PoseArray, Pose


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
   
  
def dist(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    order = np.argsort(dist_2)
    return nodes[order]

# def check_colors(img, pointsx, pointsy): 
#   traj_z = []
#   for P in range(0, len(pointsx)): 
#     x = pointsx[P]
#     y = pointsy[P]
#     B = img[y][x][0]
#     G = img[y][x][1]
#     R = img[y][x][2]
#     colors = [B, G, R]
#     color = colors.index(max(colors))
#     # print('B', img[y][x][0], 'G', img[y][x][1], 'R', img[y][x][2])
#     # print(color)
#     if color == 0: 
#       # blue z = 0.5
#       traj_z.append(0.5)
#     if color == 1: 
#       # green z = 1
#       traj_z.append(1)
#     if color == 2: 
#       # red z = 1.5
#       traj_z.append(1.5)
#   print(traj_z)

def check_colors(img, pointsx, pointsy): 
  hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
  traj_z = []
  for P in range(0, len(pointsx)): 
    x = pointsx[P]
    y = pointsy[P]
    h, _, _ = hsv[y,x]
    red_low1 = 0
    red_high1 = 15

    red_low2 = 170
    red_high2 = 180

    green_low = 30
    green_high = 80

    blue_low = 90
    blue_high = 130

    if blue_low <= h <= blue_high:
      # blue z = 0.5
      traj_z.append(0.5)
    elif green_low <= h <= green_high: 
      # green z = 1
      traj_z.append(1.0)
    elif red_low1 <= h <= red_high1 or red_low2 <= h <= red_high2:
      # red z = 1.5
      traj_z.append(1.5)
    else: 
      traj_z.append(1.0)
  return traj_z

def rescale_x(img, points): 
  _, w, _ = img.shape
  center = w/2
  room = 3
  ratio = room / w
  scaled = []
  for p in points: 
    p1 = (p - center) * ratio
    scaled.append(p1)
  return scaled

def rescale_y(img, points): 
  h, _, _ = img.shape
  room = 4
  center = h/2
  ratio = room / h
  scaled = []
  for p in points: 
    p1 = (p - center) * ratio
    scaled.append(p1)
    print(points)
  return scaled

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
    self.pause_stream = False
    self.publisher_ = self.create_publisher(PoseArray, 'poses3d', 10)
    timer_period = 0.001  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.trajectory_x = [0.0]
    self.trajectory_y = [0.0]
    self.trajectory_z = [0.0]
    self.msg = PoseArray()
   
  def listener_callback(self, data):
    if not self.pause_stream:
      self.get_logger().info('Receiving video frame')
      cf = self.br.imgmsg_to_cv2(data)
      cf = gamma_correction(cf)
      cf_cut, is_paper = find_paper(cf)
      copy_cf = cf_cut.copy()
      self.get_logger().info(f'{cf_cut.shape[0], cf_cut.shape[1]}')
      if is_paper:
        grey = cv2.cvtColor(cf_cut.copy(), cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(grey, 30, 0.01, 10)
        if corners is not None:
          corners = corners.astype(int)
          corners = [c for c in corners if dist_from_center(cf_cut, c)]
          # corners = np.unique(corners, axis=0)
          self.get_logger().info(f'unique: {corners}')
          if len(corners) > 1:
            corners = np.intp(corners)
            corners = np.reshape(corners, (-1, 2))
            h, w, _ = cf_cut.shape
            corners = dist([w/2,h/2], corners)
            # print('dist', type(corners), corners.shape)
            copy_corners = corners
            self.trajectory_x = []
            self.trajectory_y = []
            cor = corners[0].tolist()
            self.trajectory_x.append(cor[0])
            self.trajectory_y.append(cor[1])
            for i in corners:
                x, y = i.ravel()
                res = dist(i, copy_corners)[:2]
                cv2.circle(cf_cut, (x, y), 3, 255, -1)
                if res.shape[0] == 2:
                  self.trajectory_x.append(res[1][0])
                  self.trajectory_y.append(res[1][1])
                print(res.shape)
                for p in res:
                    xd, yd = p.ravel()
                    cv2.line(cf_cut,(x,y), (xd,yd), [255,0,0], 1)
                copy_corners = np.delete(copy_corners,0,0)
            key = cv2.waitKey(1)
            if key == ord('q'):
              self.pause_stream = True
              self.trajectory_z = check_colors(copy_cf, self.trajectory_x, self.trajectory_y)
              print(type(self.trajectory_z[0]))
              self.trajectory_x = rescale_x(cf_cut, self.trajectory_x)
              self.trajectory_y = rescale_y(cf_cut, self.trajectory_y)
              self.get_logger().info(f'Paused stream processing')
              self.msg = PoseArray()
              for i in range(0, len(self.trajectory_x)):
                  x, y, z, qx, qy, qz, qw = self.trajectory_x[i], self.trajectory_y[i], self.trajectory_z[i], 0.0, 0.0, 0.0, 1.0 # set Pose values
                  pose = Pose() # create a new Pose message
                  pose.position.x, pose.position.y, pose.position.z = x, y, z
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
                  self.msg.poses.append(pose) # add the new Pose object to the PoseArray list
              # self.publisher_.publish(msg)
              # self.get_logger().info(f'Publishing path : {msg}')
        cv2.imshow('lines', cf_cut)
      cv2.imshow("camera", cf)
    
  def timer_callback(self):
    # msg = PoseArray()
    # # self.get_logger().info(f"{msg}" )

    # msg.header.stamp = self.get_clock().now().to_msg()
    # # x_list = [0.6, 0.6, 1.2, 1.2, 1.8]
    # # y_list = [0.0, 0.6, 0.6, -0.6, 0.0]
    # # x_list = [0.6, 1.2, 1.8, 1.8, 2.4]
    # # y_list = [0.0, -1.0, 1.6, 0.0, 0.0]
    # # 
    # for i in range(0, len(self.trajectory_x)):
    #     x, y, z, qx, qy, qz, qw = self.trajectory_x[i], self.trajectory_y[i], self.trajectory_z[i], 0.0, 0.0, 0.0, 1.0 # set Pose values
    #     pose = Pose() # create a new Pose message
    #     pose.position.x, pose.position.y, pose.position.z = x, y, z
    #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    #     msg.poses.append(pose) # add the new Pose object to the PoseArray list
    # # for i in range(5):
    # #     x, y, z, qx, qy, qz, qw = x_list[i], y_list[i], 0.0, 0.0, 0.0, 0.0, 1.0 # set Pose values
    # #     pose = Pose() # create a new Pose message
    # #     pose.position.x, pose.position.y, pose.position.z = x, y, z
    # #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
    # #     msg.poses.append(pose) # add the new Pose object to the PoseArray list
    self.publisher_.publish(self.msg)
    self.get_logger().info(f'Publishing path : {self.msg}')
  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()