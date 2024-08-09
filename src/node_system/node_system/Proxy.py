from library.Constants import Constants
from message.srv import CameraImage


import numpy as np


class Proxy():
    def __init__(self):
        self.image = np.empty(shape=[1])

    def get_image(self):
        return self.image

    def set_image(self, image):
        self.image = image
