import traceback
import cv2
from cv_bridge import CvBridge

class Video:
    def __init__(self):
        self.cam = None
        self.bridge = CvBridge()

    def open(self):
        try:
            self.cam = cv2.VideoCapture(0)
            self.getlogger().info('Camera Opened')
            # if not self.cam.isOpened():
            #     print("Failed to open camera")
            #     return False
            # return True
        except Exception as error:
            print(f"Error opening camera: {error}, {type(error)=}")
            print(traceback.format_exc())
    
    def close(self):
        try:
            self.cam.release()
            cv2.destroyAllWindows()
            self.getlogger().info('Camera Closed')
        except Exception as error:
            print(f"Error closing camera: {error}, {type(error)=}")
            print(traceback.format_exc())
    
    def read(self):
        try:
            success, frame = self.cam.read()
            if success:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                return img_msg
            self.getlogger().info('Read Image')
        except Exception as error:
            print(f"Error reading camera: {error}, {type(error)=}")
            print(traceback.format_exc())