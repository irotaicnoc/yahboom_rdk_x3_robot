# import cv2
import os
import numpy as np
from matplotlib import pyplot as plt

img_name = '1_03_frame_reshaped.npy'
# folder_path = 'C:/Users/Marco/GIT/yahboom_rdk_x3_robot/src/camera_pub/camera_pub/imgs/640_480/'
folder_path = 'C:/Users/Marco/GIT/yahboom_rdk_x3_robot/src/camera_pub/camera_pub/imgs/1920_1080/'
img_path = folder_path + img_name
np_array = np.load(img_path)
print(np_array.shape)
plt.imshow(np_array, interpolation='nearest')
plt.show()
