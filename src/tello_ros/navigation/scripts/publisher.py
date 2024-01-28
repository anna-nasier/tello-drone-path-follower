#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from gazebo_msgs.msg import ModelStates
import socket
import struct
import numpy as np
import json

def get_quaternion_from_euler(roll, yaw, pitch):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in degrees.
      :param pitch: The pitch (rotation around y-axis) angle in degrees.
      :param yaw: The yaw (rotation around z-axis) angle in degrees.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    roll = np.deg2rad(roll)
    yaw = np.deg2rad(yaw)
    pitch = np.deg2rad(pitch)
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'poses3d', 10)
        self.publisher_gz = self.create_publisher(ModelStates, '/gazebo/model_states', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.simulation = True
        
        if not self.simulation:
            # values
            ip = "192.168.110.2"
            port = 12111

            # create socket
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            # allow multiple use of the same socket
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # bind socket
            self.s.bind((ip,port))



    def timer_callback(self):
        # msg = PoseArray()
        # self.get_logger().info(f"{msg}" )

        # msg.header.stamp = self.get_clock().now().to_msg()
        # # x_list = [0.6, 0.6, 1.2, 1.2, 1.8]
        # # y_list = [0.0, 0.6, 0.6, -0.6, 0.0]
        # x_list = [0.6, 1.2, 1.8, 1.8, 2.4]
        # y_list = [0.0, -1.0, 1.6, 0.0, 0.0]
        # # 
        # for i in range(5):
        #     x, y, z, qx, qy, qz, qw = x_list[i], y_list[i], 0.0, 0.0, 0.0, 0.0, 1.0 # set Pose values
        #     pose = Pose() # create a new Pose message
        #     pose.position.x, pose.position.y, pose.position.z = x, y, z
        #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        #     msg.poses.append(pose) # add the new Pose object to the PoseArray list
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing path : ')
        
        if not self.simulation: #jezeli nie symulacja to pobieramy dane
            data = self.s.recvfrom(1024)[0]
            data1 = json.loads(data)
            # recieve data
                
            QDrone_values = data1["Tello"]
            self.get_logger().info(f'Recived data :{QDrone_values} ')
            # Przypisanie wartości do osobnych zmiennych
            a, b, c, d, e, f, g, _ = QDrone_values

            widocznosc = a
            x = b
            y = c
            z = d
            roll = e
            pitch = f
            yaw = g

            quat = get_quaternion_from_euler(roll, pitch, yaw)
            qx = quat[0]
            qy = quat[1]
            qz = quat[2]
            qw = quat[3]

            self.get_logger().info(f'x = {x}, y = {y}, z = {z} \n qx = {qx}, qy = {qy}, qz = {qz}, qw = {qw} \n ')

            msg_gz = ModelStates()

            if widocznosc:
                pose_gz = Pose() # create a new Pose message

                pose_gz.position.x = float(x)
                pose_gz.position.y = float(y)
                pose_gz.position.z = float(z)
                pose_gz.orientation.x = float(qx)
                pose_gz.orientation.y = float(qy)
                pose_gz.orientation.z = float(qz)
                pose_gz.orientation.w = float(qw)

                # self.get_logger().info(f"pose gz {pose_gz}" )

                msg_gz.pose = [pose_gz]
                self.publisher_gz.publish(msg_gz)



      

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



if __name__ == '__main__':
    main()