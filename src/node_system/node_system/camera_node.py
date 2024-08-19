import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class Camera_node(Node):
    
    def __init__(self):
        super().__init__('camera_node')
        self.img_publisher = self.create_publisher(Image, 'img_processed', 10)
        timer_period = 0.1 #1초마다 한번 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        # OpenCv로 이미지 읽어오기
        # 실제 응용에서는 카메라에서 이미지를 읽어와야함
        ret, frame = self.cap.read()
        if ret == True:
            self.img_publisher.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('프레임 보냄')


def main(args=None):
  rp.init(args=args)
  image_publisher = Camera_node()
  rp.spin(image_publisher)
  image_publisher.destroy_node()
  rp.shutdown()

if __name__ == '__main__':
    main()