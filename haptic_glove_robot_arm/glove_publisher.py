# This script receives glove data from a socket connection and publishes it to a ROS 2 topic.
# It is designed to work with the haptic glove hardware, which sends data over a TCP socket.
# The glove data includes finger positions and tilt angles, which are parsed and published
# to the 'glove_data' topic. The script uses a timer to periodically check for incoming data
# and publish it. The socket connection is established on a specified IP address and port.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class GlovePublisher(Node):
    def __init__(self):
        super().__init__('glove_publisher')

        self.publisher_ = self.create_publisher(String, 'glove_data', 10)

        # socket server
        self.server_ip = '0.0.0.0'  # 
        self.server_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.server_ip, self.server_port))
        self.sock.listen(1)
        self.get_logger().info(f'Waiting for connection on {self.server_ip}:{self.server_port}...')
        self.conn, self.addr = self.sock.accept()
        self.get_logger().info(f'Connected by {self.addr}')

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        try:
            data = self.conn.recv(1024)  
            if data:
                decoded = data.decode().strip()
                for line in decoded.split('\n'):
                    if line:
                        self.publisher_.publish(String(data=line))
                        self.get_logger().info(f'Published: {line}')
        except Exception as e:
            self.get_logger().error(f'Error receiving glove data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GlovePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.conn.close()
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()