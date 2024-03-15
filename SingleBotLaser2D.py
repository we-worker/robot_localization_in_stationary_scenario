import numpy as np
import random
import utils
import cv2
import math

from concurrent.futures import ThreadPoolExecutor


class SingleBotLaser2Dgrid:
    def __init__(self, bot_pos, bot_param, fname):
        self.bot_pos = bot_pos
        self.bot_param = bot_param
        self.img_map = None

        scale = 1
        img = utils.Image2Map(fname)
        img = cv2.resize(img, (round(scale*img.shape[1]), round(scale*img.shape[0])), interpolation=cv2.INTER_LINEAR)
        self.img_map = img
    
   
    def Sensor(self):
        sense_data = []
        inter = (self.bot_param[2] - self.bot_param[1]) / (self.bot_param[0])
        for i in range(self.bot_param[0]):
            theta = self.bot_pos[2] + self.bot_param[1] + i*inter
            sense_data.append(self.RayCast(np.array((self.bot_pos[0], self.bot_pos[1])), theta))
        return sense_data

