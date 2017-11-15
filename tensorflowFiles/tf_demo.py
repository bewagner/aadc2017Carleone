# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""Simple image classification with Inception.

This demo script is based upon the TensorFlow tutorial, the source can be found at:
https://github.com/tensorflow/models/tree/master/tutorials/image/imagenet

Therefore the original description is provided and extended by the adopted description.

original description:
Run image classification with Inception trained on ImageNet 2012 Challenge data
set.

This program creates a graph from a saved GraphDef protocol buffer,
and runs inference on an input JPEG image. It outputs human readable
strings of the top 5 predictions along with their probabilities.

Change the --image_file argument to any jpg image to compute a
classification of that image.

Please see the tutorial and website for a detailed description of how
to use this script to perform image recognition.

https://tensorflow.org/tutorials/image_recognition/

adopted demo description:
This demo script is to be used with TensorFlow 1.1 (with or without GPU support) and
Thrift 0.93+.

It basically wraps up the original image classification model and provides a
parameterizable config and functions for thrift interaction for the ease of use.

With the default settings, the script tries to classify the provided image and
provides the five most probable classifications in a human readable form (String)
and the corresponding probabilities.

The results are packed into a list and prepared to be sent back by thrift.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from ExtIf import ttypes
from ExtIf.ttypes import *

import os.path
import re

import numpy as np
import tensorflow as tf

"""basic configuration for this TensorFlow demo script"""
config = {
    "model_path": 'model',                          				# folder the model resides in
    "graph_name": 'classify_image_graph_def.pb',    				# name of the model graph file
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
tf_session = None

class NodeLookup(object):
    """Converts integer node ID's to human readable labels."""
    """Taken from the TensorFlow tutorial - unchanged"""

    def __init__(self, label_lookup_path=None, uid_lookup_path=None):
        if not label_lookup_path:
            label_lookup_path = os.path.join(config['model_path'], config['label_file'])

        if not uid_lookup_path:
            uid_lookup_path = os.path.join(config['model_path'], config['readable_label_file'])

        self.node_lookup = self.load(label_lookup_path, uid_lookup_path)

    def load(self, label_lookup_path, uid_lookup_path):
        """Loads a human readable English name for each softmax node.

        Args:
        label_lookup_path: string UID to integer node ID.
        uid_lookup_path: string UID to human-readable string.

        Returns:
        dict from integer node ID to human-readable string.
        """
        if not tf.gfile.Exists(uid_lookup_path):
            tf.logging.fatal('File does not exist %s', uid_lookup_path)
        if not tf.gfile.Exists(label_lookup_path):
            tf.logging.fatal('File does not exist %s', label_lookup_path)

        # Loads mapping from string UID to human-readable string
        proto_as_ascii_lines = tf.gfile.GFile(uid_lookup_path).readlines()
        uid_to_human = {}
        p = re.compile(r'[n\d]*[ \S,]*')
        for line in proto_as_ascii_lines:
            parsed_items = p.findall(line)
            uid = parsed_items[0]
            human_string = parsed_items[2]
            uid_to_human[uid] = human_string

        # Loads mapping from string UID to integer node ID.
        node_id_to_uid = {}
        proto_as_ascii = tf.gfile.GFile(label_lookup_path).readlines()
        for line in proto_as_ascii:
            if line.startswith('  target_class:'):
                target_class = int(line.split(': ')[1])
            if line.startswith('  target_class_string:'):
                target_class_string = line.split(': ')[1]
                node_id_to_uid[target_class] = target_class_string[1:-2]

        # Loads the final mapping of integer node ID to human-readable string
        node_id_to_name = {}
        for key, val in node_id_to_uid.items():
            if val not in uid_to_human:
                tf.logging.fatal('Failed to locate: %s', val)
            name = uid_to_human[val]
            node_id_to_name[key] = name

        return node_id_to_name

    def id_to_string(self, node_id):
        if node_id not in self.node_lookup:
            return ''
        return self.node_lookup[node_id]

def tf_demo_init():
    """
    initializes the TensorFlow model and session
    
    This initialization function is to be called only once and before trying to
    classify an image.
    """
    # initialize graph
    create_graph()

    # initialize TensorFlow Session
    init_tf_session()


def create_graph():
    """Creates a graph from saved GraphDef file and returns a saver."""
    # introduce global config dictionary
    global config

    # Create graph
    with tf.gfile.FastGFile(os.path.join(config["model_path"], config["graph_name"]), 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        _ = tf.import_graph_def(graph_def, name='')


def init_tf_session():
    """initializes the TensorFlow session"""
    # introduce global cenfig dictionary and the session container
    global tf_session
    global config

    # initialize TensorFlow session.
    tf_session = tf.Session()

    # Some useful tensors:
    # 'softmax:0': A tensor containing the normalized prediction across
    # 1000 labels.
    # 'pool_3:0': A tensor containing the next-to-last layer containing 2048
    #   float description of the image.
    # 'DecodeJpeg/contents:0': A tensor containing a string providing JPEG
    #   encoding of the image.
    # Runs the softmax tensor by feeding the image_data as input to the graph.


def classify_image(image_data):
    """Creates a graph from saved GraphDef file and returns a saver."""
    # introduce global config dictionary
    global tf_session

    # predict
    softmax_tensor = tf_session.graph.get_tensor_by_name('softmax:0')
    predictions = tf_session.run(softmax_tensor,
                           {'DecodeJpeg/contents:0': image_data})

    # Remove single-dimensional entries from the shape of an array
    predictions = np.squeeze(predictions)

    # Creates node ID --> English string lookup.
    node_lookup = NodeLookup()

    # sorts the predictions and reduces the result to the configured number of predictions
    top_k = predictions.argsort()[-config['number_of_top_predictions']:][::-1]

    # create empty result list
    result_list = []

    # process each of the classifications
    for node_id in top_k:
        human_string = node_lookup.id_to_string(node_id)
        score = predictions[node_id]

        # append results to result list
        result_list.append(TDataResult(human_string, score))

    return result_list


def load_image_from_file(image):
    """checks if the provided image file exists, reads and returns it"""
    if not tf.gfile.Exists(image):
        tf.logging.fatal('File does not exist %s', image)
    image_data = tf.gfile.FastGFile(image, 'rb').read()

    return image_data
