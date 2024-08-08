import base64
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class ImagePublisher(Node):
    def __init__(self):
            super().__init__('image_publisher')
            qos_profile = QoSProfile(depth=10)
            self.pub = self.create_publisher(Image, 'image', qos_profile)
            timer_period = 1  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.cam = cv2.VideoCapture(0)
            if not self.cam.isOpened():
                self.get_logger().error('Failed to open camera')
                exit(1)

    def timer_callback(self):
        try:
            success, frame = self.cam.read()
            if success:
                img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub.publish(img_msg)
                cv2.imshow('Camera Frame', frame)
                cv2.waitKey(1)
            self.get_logger().info('Publishing Camera Image')
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def destroy(self):
        self.cam.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        image_publisher.get_logger().info('Publish Stopped : Keyboard Interrupt (SIGINT)')
    finally:
        image_publisher.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()