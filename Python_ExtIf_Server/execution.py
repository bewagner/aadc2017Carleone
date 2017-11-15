import glob
import sys
import os
import cv2

import numpy as np
import io
from PIL import Image
from array import array

import matplotlib.pyplot as plt
from test import tf_classification_init, classify_image
from scipy import misc


tf_classification_init()
print("--- Setup succesful ---")
print("Now classifying ...")



# image = misc.imread('catPeople.jpg')
image = misc.imread('pedestrian.jpg')

# # Show image
# plt.imshow(image)
# plt.show()


result_list = classify_image(image)
print("--- Classification succesful ---")


print(result_list)
