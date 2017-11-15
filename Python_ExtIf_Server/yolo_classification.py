from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from ExtIf import ttypes
from ExtIf.ttypes import *

import os.path
import re


import base64
import io
import matplotlib.image as mpimg


import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
import time

from darkflow.net.build import TFNet
import cv2

from scipy import misc

# "model_path": 'model/ssd_mobilenet_v1_coco/',  
# "model_path": 'model/faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017/',
"""basic configuration for this TensorFlow demo script"""
config = {
    "model_path": 'model/faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017/',  # folder the model resides in
    "graph_name": 'frozen_inference_graph.pb',    				# name of the model graph file
    "number_of_top_predictions": 5,                 				# number of the most probable predictions to provide as result
    "data_dictionary_path": '',                     				# folder the data dictionary resides in
    "label_file": 'imagenet_2012_challenge_label_map_proto.pbtxt',	# name of the label file
    "readable_label_file": 'imagenet_synset_to_human_label_map.txt'	# name of the human readable label file
}

"""
TensorFlow Session container

This global variable is introduced to reduce the computing load caused by
reloading the model before each image classification.

The session container can be initialized by calling 'init_tf_session',
function and is part of the demo initialization 'tf_demo_init'.
"""

yoloNet = None
tf_session = None


def tf_classification_init():
    """
    initializes the TensorFlow model and session
    
    This initialization function is to be called only once and before trying to
    classify an image.
    """

    # initialize TensorFlow Session
    init_tf_session()



def init_tf_session():
    """initializes the TensorFlow session"""
    # introduce global cenfig dictionary and the session container
    global yoloNet
    global tf_session

    options = {"model": "cfg/tiny-yolo-voc.cfg", "load": "bin/tiny-yolo-voc.weights", "threshold": 0.1}
    # options = {"model": "cfg/yolo.cfg", "load": "bin/yolo.weights", "threshold": 0.1}
    
    yoloNet = TFNet(options)
    tf_session = tf.Session()
    
    
	
def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

def classify_image(image_data):
    """Creates a graph from saved GraphDef file and returns a saver."""
    global yoloNet
    global config
    global tf_session
    

    with tf_session.as_default():
        image = tf.image.decode_jpeg(image_data).eval()
    
    # Actual detection.
    result_list = []

    start_time = time.time()
    result = yoloNet.return_predict(image)
    print("--- %s seconds ---" % (time.time() - start_time))
    print(result)

    
    #Print the first predictions and append them to the output data
    for item in result[:5]:
    # Debug output
    #     print("topleft")    
    #     print("x:" + str(item['topleft']['x']) + ", " + "y:" + str(item['topleft']['y']) )
    #     print("confidence")    
    #     print(item['confidence'])
    #     print("bottomright")    
    #     print(item['bottomright'])
    #     print("label")    
        print(item['label'])

        result_list.append(TDataResult(item['label'],item['confidence'],item['topleft']['y'],item['topleft']['x'],item['bottomright']['y'],item['bottomright']['x']))        
    return result_list






