from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import numpy as np

import matplotlib.pyplot as plt
import os.path
import re
import base64
import io
import matplotlib.image as mpimg

import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile

from collections import defaultdict
from io import StringIO
from PIL import Image
import time

"""basic configuration for this TensorFlow demo script"""
config = {
    "object_detection_model": 'model/ssd_mobilenet_v1_coco/',  # Fast, mAP: 21. (Here bigger 
    "graph_name": 'frozen_inference_graph.pb',    				# name of the model graph file
    "number_of_top_predictions": 5,                 				# number of the most probable predictions to provide as result
    "data_dictionary_path": '',                     				# folder the data dictionary resides in
    "label_file": 'imagenet_2012_challenge_label_map_proto.pbtxt',	# name of the label file
    "readable_label_file": 'imagenet_synset_to_human_label_map.txt',	# name of the human readable label file
    "orientation_classification_model": 'model/retrained_graph.pb'	# name of the orientation classification graph
}

"""
TensorFlow Session container

This global variable is introduced to reduce the computing load caused by
reloading the model before each image classification.

The session container can be initialized by calling 'init_tf_session',
function and is part of the demo initialization 'tf_demo_init'.
"""

object_detection_tf_session = None
orientation_classification_tf_session = None
label_map = None
orientation_classification_graph = None

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
    dims_expander = tf.expand_dims(image, 0);
    resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
    normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
    sess = tf.Session()
    result = sess.run(normalized)

    return result

def orientation_label(orientation):
    return {
        0: "away",
        1: "right",
        2: "towards",
        3: "left"
    }[orientation[0].argmax(axis=0)]


def classify_image(image):
    """Creates a graph from saved GraphDef file and returns a saver."""
    global object_detection_tf_session
    global orientation_classification_tf_session
    global label_map
    global config

    # with object_detection_tf_session.as_default():
    #     image = tf.image.decode_jpeg(image_data).eval()
        
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

    start = time.time()
    with object_detection_tf_session.as_default() as sess:
        (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],feed_dict={image_tensor: image_expanded})
    end = time.time()
    print("Needed " + str(end - start) + "ms for object detection.")
    

    # Remove single-dimensional entries from the shape of an array
    scores = np.squeeze(scores)
    boxes = np.squeeze(boxes)
    classes = np.squeeze(classes)
    
    
    #Print the first two predictions and append them to the output data
    for i in range(config['number_of_top_predictions']):
        # Enable this for debug output
        # print(str(i) + ":")
        # print(label_map[classes[i]])
        # print(scores[i])
        # print([boxes[i][0]])
        # print([boxes[i][1]])
        # print([boxes[i][2]])
        # print([boxes[i][3]])

        # Extract the current classification specifications
        label = label_map[classes[i]]
        confidence = scores[i]
        ymin = boxes[i][0]
        xmin = boxes[i][1]
        ymax = boxes[i][2]
        xmax = boxes[i][3]

        # If we found a person we additionally have to classify its orientation
        if label_map[classes[i]] == "person":
            # Crop the image to the person we found
            currentCrop = crop_image(image,ymin,xmin,ymax,xmax)
            

            
            orientation = "None"
            start = time.time()
            with orientation_classification_tf_session.as_default() as sess:
                image_tensor = read_tensor_from_image(currentCrop)
                input_operation = sess.graph.get_operation_by_name("import/input")

                
                output_operation = sess.graph.get_operation_by_name("import/final_result")
                
                orientation = sess.run(output_operation.outputs[0], {input_operation.outputs[0]: image_tensor})
                
                print("orientation:" + orientation_label(orientation))
                plt.imshow(currentCrop)
                plt.show()
                
            end = time.time()
            # print("Needed " + str(end - start) + "ms for orientation classification.")

            result_list.append(label + "|" + str(confidence) + "|" + orientation_label(orientation))
            
        else:
            result_list.append(label + "|" + str(confidence))

    return result_list

