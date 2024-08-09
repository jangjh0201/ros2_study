from library.Constants import Constants
from node_system.Proxy import Proxy
from node_system.Video import Video
from message.srv import CameraImage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


bridge = CvBridge()

class ProxyNode(Node):
    def __init__(self):
        super().__init__(Constants.PROXY)
        
        # 비즈니스 로직 객체 생성
        self.proxy = Proxy()
        self.video = Video()
        
        # 로거 설정
        qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)

        # 비디오 Topic 수신
        self.subscriber = self.create_subscription(
            Image, 
            'video', 
            self.callback_video, 
            qos_profile)
        self.image = np.empty(shape=[1])

        # 이미지 Service 
        self.client = self.create_client(CameraImage, 'service')
        self.request = CameraImage.Request()
        self.future = None
        self.service_available = False
        
        # 이미지 Service 사용 가능 여부 확인
        self.service_timer = self.create_timer(1.0, self.check_service_availability)

        # 이미지 Service 타이머 변수
        self.timer = self.create_timer(timer_period_sec=Constants.TIMER_PERIOD, callback=self.timer_execute)

    def check_service_availability(self):
        if not self.service_available:
            if self.client.wait_for_service(timeout_sec=Constants.TIMER_PERIOD):
                self.service_available = True
                self.get_logger().info('Service is now available')

    def callback_video(self, data):
        try:
            img_msg = self.video.show_video(data)
            self.get_logger().info(f"Received Image[0][0]: {img_msg[0][0]}")
            self.proxy.set_image(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def send_request(self):
        self.request.data = ["CameraImage", "Image"]
        self.future = self.client.call_async(self.request)


    def timer_execute(self):
        if self.service_available:
            if self.future is None or self.future.done():
                if self.future is not None:
                    try:
                        response = self.future.result()
                    except Exception as e:
                        self.get_logger().info(f"Service call failed {e}")
                    else:
                        self.get_logger().info(f"Result : {response}")
                
                self.send_request()

def main(args=None):
    rclpy.init(args=args)
    proxy = ProxyNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(proxy, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    proxy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
