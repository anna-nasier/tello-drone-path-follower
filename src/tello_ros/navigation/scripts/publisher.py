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
        # gazebo_msgs::msg::ModelStates>("/gazebo/model_states"
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.simulation = True
        # def optitrack(queue: Queue, run_process: Value):
        # with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        #     s.bind(('0.0.0.0', int(CLIENT_PORT)))
        
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

            # add socket to multicast group
            # self.s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)



    def timer_callback(self):
        msg = PoseArray()
        # self.get_logger().info(f"{msg}" )

        msg.header.stamp = self.get_clock().now().to_msg()
        # x_list = [0.6, 0.6, 1.2, 1.2, 1.8]
        # y_list = [0.0, 0.6, 0.6, -0.6, 0.0]
        x_list = [0.6, 1.2, 1.8, 1.8, 2.4]
        y_list = [0.0, -1.0, 1.6, 0.0, 0.0]
        # 
        for i in range(5):
            x, y, z, qx, qy, qz, qw = x_list[i], y_list[i], 0.0, 0.0, 0.0, 0.0, 1.0 # set Pose values
            pose = Pose() # create a new Pose message
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            msg.poses.append(pose) # add the new Pose object to the PoseArray list
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing path : ')
        
        if not self.simulation: #jezeli nie symulacja to pobieramy dane
            data = self.s.recvfrom(1024)[0]
            data1 = json.loads(data)
            # recieve data
            # data, addr = s.recvfrom(10240)
                
            # print data
            # data= {'rigid_bodies': 1, 'QDrone': [1, -0.545882, 0.010438, 0.115418, -0.001515, -0.000244, -0.000377]}
            QDrone_values = data1["Tello"]
            self.get_logger().info(f'Recived data :{QDrone_values} ')
            # Przypisanie wartości do osobnych zmiennych
            # Przypisanie wartości do osobnych zmiennych
            a, b, c, d, e, f, g, _ = QDrone_values
            # [a, b, c, d, e, f, g, h, i, j, k, l] = struct.unpack('ffffffffffff', data)
            # self.get_logger().info(f" a {a}")
            # self.get_logger().info(f" b {b}")

            widocznosc = a
            x = b
            y = c
            z = d
            roll = e
            pitch = f
            yaw = g
            # roll = -h
            # yaw = i
            # pitch = -j
            # bodyID = k
            # framecount = l

            quat = get_quaternion_from_euler(roll, pitch, yaw)
            qx = quat[0]
            qy = quat[1]
            qz = quat[2]
            qw = quat[3]

            self.get_logger().info(f'x = {x}, y = {y}, z = {z} \n qx = {qx}, qy = {qy}, qz = {qz}, qw = {qw} \n ')

            msg_gz = ModelStates()
            # print(msg_gz)
            # self.get_logger().info(f"{msg_gz}" )
            # self.get_logger().info(f"{msg_gz.pose}" )

            if widocznosc:
                # msg_gz.pose
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


    # minimal_publisher = MinimalPublisher()

    # rclpy.spin(minimal_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()