from message.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.srv = self.create_service(AddTwoInts, "service", self.callback)

    def callback(self, req, res):
        res.sum = req.a + req.b
        self.get_logger().info(f"Request: {req.a} + {req.b} = {res.sum}")

        return res
    
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()