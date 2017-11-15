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
import scipy.misc

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

# "object_detection_model": 'model/ssd_mobilenet_v1_coco/',  # Fast, mAP: 21. (Here bigger values are better).
# "object_detection_model": 'model/ssd_inception_v2_coco_11_06_2017/', # Fast. mAP: 24.
# "object_detection_model": 'model/rfcn_resnet101_coco_11_06_2017/', # Medium. mAP: 30.
# "object_detection_model": 'model/faster_rcnn_resnet101_coco_11_06_2017/', # Medium. mAP: 32.
# "object_detection_model": 'model/faster_rcnn_inception_resnet_v2_atrous_coco_11_06_2017/', # Slow. mAP: 37.


"""basic configuration for this TensorFlow demo script"""
config = {
    "object_detection_model": 'model/rfcn_resnet101_coco_11_06_2017/', # Medium. mAP: 30.
    "graph_name": 'frozen_inference_graph.pb',    				# name of the model graph file
    "number_of_top_predictions": 5,                 				# number of the most probable predictions to provide as result
    "data_dictionary_path": '',                     				# folder the data dictionary resides in
    "label_file": 'imagenet_2012_challenge_label_map_proto.pbtxt',	# name of the label file
    "readable_label_file": 'imagenet_synset_to_human_label_map.txt',	# name of the human readable label file
    "orientation_classification_model": 'model/retrained_graphTwoDirections.pb'	# name of the orientation classification graph    
}


"""
TensorFlow Session container

This global variable is introduced to reduce the computing load caused by
reloading the model before each image classification.

The session container can be initialized by calling 'init_tf_session',
function and is part of the demo initialization 'tf_demo_init'.
"""

object_detection_tf_session = None
orientation_graph = None
label_map = None


def load_label_map_from_file(file_path):
    global label_map
    with open(file_path) as file: 
        label_map = {}
        for line in file:
            line = line.strip()
            if(line[:12] == "display_name"):
                label_map[lastID] = str(line[15:-1])
            if(line[:2] =="id"):
                lastID = int(line[4:])

def tf_classification_init():
    """
    initializes the TensorFlow model and session
    
    This initialization function is to be called only once and before trying to
    classify an image.
    """
    # initialize graph
    create_graph()

    # initialize TensorFlow Session for object detection
    init_object_detection_session()

    # initialize TensorFlow Session for orientation classification
    init_orientation_classification_session()

def create_graph():
    """Creates a graph from saved GraphDef file and returns a saver."""
    # introduce global config dictionary
    global config

    PATH_TO_CKPT = config['object_detection_model'] + config['graph_name']
    
    # Create graph
    detection_graph = tf.Graph()
    
    
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
	serialized_graph = fid.read()
	od_graph_def.ParseFromString(serialized_graph)
	tf.import_graph_def(od_graph_def, name='')
        
        
def load_graph(model_file):
    graph = tf.Graph()
    graph_def = tf.GraphDef()

    with open(model_file, "rb") as f:
        graph_def.ParseFromString(f.read())
        with graph.as_default():
            tf.import_graph_def(graph_def)

    return graph

def init_orientation_classification_session():
    global config
    global orientation_classification_tf_session 
    
    orientation_classification_graph = load_graph(config['orientation_classification_model'])
    orientation_classification_tf_session = tf.Session(graph = orientation_classification_graph)

def init_object_detection_session():
    """initializes the TensorFlow session"""
    # introduce global cenfig dictionary and the session container
    global object_detection_tf_session
    global config
    global label_map

    # initialize TensorFlow session.
    # with detection_graph.as_default():
    object_detection_tf_session = tf.Session()

    # Load the label map
    load_label_map_from_file("mscoco_label_map.pbtxt")
    	
def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

def crop_image(image,ymin,xmin,ymax,xmax):
    width = image.shape[1]
    height = image.shape[0]
    return image[int(np.floor(ymin*height)) : int(np.floor(ymax*height)), int(np.floor(xmin*width)) : int(np.floor(xmax*width)),:]    

def read_tensor_from_image(image, input_height=224, input_width=224,
			   input_mean=128, input_std=128):
    global object_detection_tf_session
    
    img = Image.fromarray(image) 
    img = img.resize((input_width,input_height), Image.BICUBIC)
    img = np.array(img)

    # img = (img - np.mean(img)) / np.std(img)
    
    # Save image for debug purposes
    # scipy.misc.imsave('person.jpg',img)

    return np.expand_dims(img,0)


def orientation_label(orientation):
    global config
    if config["orientation_classification_model"] == 'model/retrained_graphFourDirections.pb':
        return {
            0: "away",
            1: "right",
            2: "towards",
            3: "left"
        }[orientation[0].argmax(axis=0)]
    else:
        return {
            0: "right",
            1: "left"
        }[orientation[0].argmax(axis=0)]

def classify_image(image_data):
    """Creates a graph from saved GraphDef file and returns a saver."""
    global object_detection_tf_session
    global orientation_classification_tf_session
    global label_map
    global config

    with object_detection_tf_session.as_default():
        image = tf.image.decode_jpeg(image_data).eval()
    
    # Actual Detection
    # # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_expanded = np.expand_dims(image, axis=0)
    
    image_tensor = object_detection_tf_session.graph.get_tensor_by_name('image_tensor:0')

    # # Each box represents a part of the image where a particular object was detected.
    boxes = object_detection_tf_session.graph.get_tensor_by_name('detection_boxes:0')
    
    # # Each score represent how level of confidence for each of the objects.
    # # Score is shown on the result image, together with the class label.
    scores = object_detection_tf_session.graph.get_tensor_by_name('detection_scores:0')
    classes = object_detection_tf_session.graph.get_tensor_by_name('detection_classes:0')
    num_detections = object_detection_tf_session.graph.get_tensor_by_name('num_detections:0')
    
    # Actual detection.
    result_list = []

    
    # Wether to print the time needed for classifying
    printTime = True

    if printTime:
        start = time.time()
        
    (boxes, scores, classes, num_detections) = object_detection_tf_session.run(
        [boxes, scores, classes, num_detections],
        feed_dict={image_tensor: image_expanded})
    
    if printTime:
        end = time.time()
        print(end - start)
    

    # Remove single-dimensional entries from the shape of an array
    scores = np.squeeze(scores)
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes)

    numberOfPersonsToClassify = 1
    personsClassified = 0

    
    #Print the first two predictions and append them to the output data
    for i in range(config['number_of_top_predictions']):
        # Extract the current classification specifications
        label = label_map[classes[i]]
        confidence = scores[i]
        ymin = boxes[i][0]
        xmin = boxes[i][1]
        ymax = boxes[i][2]
        xmax = boxes[i][3]
        
        with orientation_classification_tf_session.as_default() as sess:
            input_operation = sess.graph.get_operation_by_name("import/input")
            output_operation = sess.graph.get_operation_by_name("import/final_result")


        kidThreshold = 0.2
            
        # if ymin > kidThreshold and label in ["person", "teddy bear", "toothbrush", "skateboard"]:
        #     result_list.append(TDataResult(label, confidence, ymin, xmin, ymax, xmax, "None", True))            
            
        # If we found a person we additionally have to classify its orientation
        if personsClassified < numberOfPersonsToClassify and label == "person":
            # elif personsClassified < numberOfPersonsToClassify and label == "person":
            personsClassified += 1
            
            # Crop the image to the person we found
            currentCrop = crop_image(image,ymin,xmin,ymax,xmax)

            # Save the current crop for debug purposes
            # scipy.misc.imsave('person' + str(personsClassified) +'.jpg',currentCrop)
            
            orientation = "None"

            if printTime:
                start = time.time()
                
            with orientation_classification_tf_session.as_default() as sess:
                image_tensor = read_tensor_from_image(currentCrop)
                orientation = sess.run(output_operation.outputs[0], {input_operation.outputs[0]: image_tensor})
                orientationString = orientation_label(orientation)
            
            if printTime:
                end = time.time()
                print("Needed " + str(end - start) + "ms for orientation classification.")

            result_list.append(TDataResult(label,confidence,ymin,xmin,ymax,xmax,orientationString,False))            
        else:
            # Otherwise we just return the classification with a default orientation of "None"
            result_list.append(TDataResult(label,confidence,ymin,xmin,ymax,xmax,"None",False))            

    return result_list


