import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from library.Constants import Constants
from sensor_msgs.msg import Image
from node_system.Video import Video
import cv2

class VideoNode(Node):
    def __init__(self):
            super().__init__(Constants.VIDEO)

            # 카메라 연결
            self.video = Video()
            self.video.open()

            # 로거 설정
            qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)

            # 비디오 Topic 송신
            self.publisher = self.create_publisher(Image, 'video', qos_profile=qos_profile)
            
            # 비디오 Topic 타이머 변수
            self.timer = self.create_timer(timer_period_sec=Constants.TIMER_PERIOD, callback=self.timer_execute)


    def timer_execute(self):
        try:
            img_msg = self.video.get_video()
            self.get_logger().info(f'Publishing Video')
            
            self.publisher.publish(img_msg)
            
        except Exception as error:
            self.get_logger().error(f"An error occurred: {error}")

    def destroy(self):
        try:
            self.video.close()
        except Exception as error:
            self.get_logger().error(f"An error occurred: {error}")

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_node = VideoNode()

    try:
        rclpy.spin(video_node)
    except KeyboardInterrupt:
        video_node.get_logger().info('Publish Stopped : Keyboard Interrupt (SIGINT)')
    finally:
        video_node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()