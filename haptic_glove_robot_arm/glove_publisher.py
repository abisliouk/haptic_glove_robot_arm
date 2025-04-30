# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import serial
# import time

# class GlovePublisher(Node):
#     def __init__(self):
#         super().__init__('glove_publisher')
#         self.publisher_ = self.create_publisher(String, 'glove_data', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
#         self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update port if necessary
#         time.sleep(2)

#     def timer_callback(self):
#         try:
#             line = self.ser.readline().decode().strip()
#             self.publisher_.publish(String(data=line))
#             self.get_logger().info(f'Published: {line}')
#         except Exception as e:
#             self.get_logger().error(f'Error reading glove data: {e}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GlovePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



# MOCKED GLOVE DATA PUBLISHER

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import random
# import time

# class GlovePublisher(Node):
#     def __init__(self):
#         super().__init__('glove_publisher')
#         self.publisher_ = self.create_publisher(String, 'glove_data', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

#     def timer_callback(self):
#         # Mocked glove data
#         fingers = [random.randint(0, 100) for _ in range(5)]
#         pitch = random.uniform(-30.0, 30.0)
#         roll = random.uniform(-30.0, 30.0)

#         # Format the data as a string similar to the glove's output
#         finger_data = ",".join([f"{i+1}:{fingers[i]}" for i in range(5)])
#         tilt_data = f"{pitch:.2f},{roll:.2f}"
#         mocked_data = f"FINGER:{finger_data}|TILT:{tilt_data}"

#         # Publish the mocked data
#         self.publisher_.publish(String(data=mocked_data))
#         self.get_logger().info(f'Published Mocked Data: {mocked_data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = GlovePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


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
