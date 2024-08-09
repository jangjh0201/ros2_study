import traceback
import cv2
from cv_bridge import CvBridge

class Video:
    def __init__(self):
        self.cam = None
        self.bridge = CvBridge()
        self.img_msg = None

    def open(self):
        try:
            self.cam = cv2.VideoCapture(0)
            # if not self.cam.isOpened():
            #     print("Failed to open camera")
            #     return False
            # return True
        except Exception as error:
            print(f"Error opening camera: {error}, {type(error)=}")
            print(f"Traceback: {traceback.format_exc()}")
    
    def close(self):
        try:
            self.cam.release()
            cv2.destroyAllWindows()
        except Exception as error:
            print(f"Error closing camera: {error}, {type(error)=}")
            print(f"Traceback: {traceback.format_exc()}")

    
    def get_video(self):
        '''
        비디오 촬영
        '''
        try:
            success, frame = self.cam.read()
            if success:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                
                return img_msg
        except Exception as error:
            print(f"Error reading camera: {error}, {type(error)=}")
            print(f"Traceback: {traceback.format_exc()}")

    def show_video(self, data):
        '''
        비디오 상영
        '''
        try:
            img_msg = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            return img_msg
        except Exception as error:
            print(f"Error showing camera: {error}, {type(error)=}")
            print(f"Traceback: {traceback.format_exc()}")

    def get_photo(self, img_msg):
        try:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                cv2.imwrite('image.jpg', img_msg)
        except Exception as error:
            print(f"Error taking photo: {error}, {type(error)=}")
            print(f"Traceback: {traceback.format_exc()}")
            
    