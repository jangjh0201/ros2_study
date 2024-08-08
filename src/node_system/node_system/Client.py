import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

# Server
from message.srv import AddTwoInts
import sys

# ImagePublisher
from std_msgs.msg import String

bridge = CvBridge()

class Client(Node):
    def __init__(self):
        super().__init__('client')
        
        # Server
        self.cli = self.create_client(AddTwoInts, 'service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        
        # ImagePublisher
        qos_profile = QoSProfile(depth=10)
        self.image_sub = self.create_subscription(
            Image, 
            'image', 
            self.image_callback, 
            qos_profile)
        self.image = np.empty(shape=[1])

        # Timer for periodic request
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.future = None
        self.num1 = 1
        self.num2 = 3

    def send_request(self):
        self.req.a = self.num1
        self.req.b = self.num2
        self.future = self.cli.call_async(self.req)

    def image_callback(self, data):
        try:
            self.image = bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow('Received Image', self.image)
            cv2.waitKey(1)
            self.get_logger().info(f"Received Image[0][0]: {self.image[0][0]}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")



    def timer_callback(self):
        if self.future is None or self.future.done():
            if self.future is not None:
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                else:
                    self.get_logger().info(f"Result of add_two_ints: {response.sum}")
            
            self.send_request()
            self.num1 += 1
            self.num2 += 3

def main(args=None):
    rclpy.init(args=args)
    client = Client()

    while rclpy.ok():
        rclpy.spin_once(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()