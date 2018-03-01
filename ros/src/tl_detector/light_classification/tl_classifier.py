from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor
from scipy.stats import norm

class TLClassifier(object):
    def __init__(self):
        #Load classifier
        classification_graph = 'light_classification/classification_model/output_graph.pb'
        classification_labels = 'light_classification/classification_model/output_labels.txt'
        if not tf.gfile.Exists(classification_labels):
            tf.logging.fatal('labels file does not exist %s', classification_labels)
        self.labels = self.load_labels(classification_labels)
        self.labels_dic = {
                            'red':0,
                            'yellow':1,
                            'green':2,
                            }
        self.sess = self.load_graph(classification_graph)
        

    def load_image(self, filename):
      """Read in the image_data to be classified."""
      return tf.gfile.FastGFile(filename, 'rb').read()
    
    
    def load_labels(self, filename):
      """Read in labels, one label per line."""
      return [line.rstrip() for line in tf.gfile.GFile(filename)]
    
    
    def load_graph(self, filename):       
        with tf.Session() as persisted_sess:
            with tf.gfile.FastGFile(filename, 'rb') as f:
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(f.read())
                persisted_sess.graph.as_default()
                tf.import_graph_def(graph_def, name='')
                return persisted_sess        
    def run_graph(self, image_data, labels, input_layer_name, output_layer_name,
              num_top_predictions=1):
        #with tf.Session() as sess:
        # Feed the image_data as input to the graph.
        #   predictions will contain a two-dimensional array, where one
        #   dimension represents the input image count, and the other has
        #   predictions per class
        softmax_tensor = self.sess.graph.get_tensor_by_name(output_layer_name)
        predictions, = self.sess.run(softmax_tensor, {input_layer_name: image_data, 'Placeholder:0':1.0})

        # Sort to show labels in order of confidence
        top_k = predictions.argsort()[-num_top_predictions:][::-1]
        for node_id in top_k:
            human_string = labels[node_id]
            score = predictions[node_id]
            print('%s (score = %.5f)' % (human_string, score))
        return human_string    
    
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        input_layer = 'DecodeJpeg/contents:0'
        output_layer = 'final_result:0' 
        num_top_predictions = 1
        predict_label = self.run_graph(image, self.labels, input_layer, output_layer, num_top_predictions)        
        return self.labels_dic[predict_label]
