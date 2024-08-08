import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(String, 'text', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()