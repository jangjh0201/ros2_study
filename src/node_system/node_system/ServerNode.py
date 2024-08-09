from message.srv import CameraImage

import rclpy
from rclpy.node import Node

class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.srv = self.create_service(CameraImage, "service", self.callback)

    def callback(self, req, res):
        self.get_logger().info(f"Request: {req}")
        self.get_logger().info(f"Response: {req}")

        res.message = "Server request received"
        return res
    
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()