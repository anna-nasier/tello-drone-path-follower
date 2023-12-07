import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from camsub.find_paper_traj import find_paper
from camsub.line import Line3D

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
    current_frame = self.br.imgmsg_to_cv2(data)
    cf = current_frame.copy()
    cf_cut, is_paper = find_paper(cf)
    self.get_logger().info(f"Paper found: {is_paper}")
    grey = cv2.cvtColor(cf_cut.copy(), cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(grey, 50, 200, None, 3)
    linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cf_cut, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

    cv2.imshow('lines', cf_cut)
    # cv2.imshow("camera", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()