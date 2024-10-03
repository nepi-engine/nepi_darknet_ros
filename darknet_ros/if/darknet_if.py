#!/usr/bin/env python

import sys
import os
import os.path

import glob
import subprocess
import yaml
import time
import rospy
import numpy as np


from nepi_edge_sdk_base import nepi_ros


from std_msgs.msg import Empty, Float32
from nepi_ros_interfaces.msg import ObjectCount
from nepi_ros_interfaces.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse


from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


AI_NAME = 'Darknet' # Use in display menus
FILE_TYPE = 'AIF'
AI_DICT = dict(
    description = 'Darknet ai framework support',
    pkg_name = 'nepi_darknet_ros',
    class_name = 'DarknetAIF',
    node_file = 'nepi_darknet_ros',
    node_name = 'nepi_darknet_ros',
    launch_file = 'darknet_ros.launch',
    model_prefix = 'darknet_',
    models_path = '/mnt/nepi_storage/ai_models/darknet_ros'
)

class DarknetAIF(object):
    def __init__(self, ai_dict,node_namespace):
      if node_namespace[-1] == "/":
        node_namespace = node_namespace[:-1]
      self.node_namespace = node_namespace
      self.pkg_name = ai_dict['pkg_name']
      self.node_name = ai_dict['node_name']
      self.launch_file = ai_dict['launch_file']
      self.model_prefix = ai_dict['model_prefix']
      self.model_search_path = ai_dict['models_path']
      threshold_namespace = self.node_namespace + '/nepi_darknet_ros/set_threshold'
      self.set_threshold_pub = rospy.Publisher(threshold_namespace, Float32, queue_size=1, latch=True)
    
    #################
    # Darknet Model Functions

    def getModelsDict(self):
        models_dict = dict()
        classifier_name_list = []
        classifier_size_list = []
        classifier_classes_list = []
        # Try to obtain the path to Darknet models from the system_mgr
        cfg_path_config_folder = os.path.join(self.model_search_path, 'config')
        rospy.loginfo("Darknet looking for models config files in folder: " + cfg_path_config_folder)
        # Grab the list of all existing darknet cfg files
        self.cfg_files = glob.glob(os.path.join(cfg_path_config_folder,'*.yaml'))
        # Remove the ros.yaml file -- that one doesn't represent a selectable trained neural net
        try:
            self.cfg_files.remove(os.path.join(cfg_path_config_folder,'ros.yaml'))
        except:
            rospy.logwarn("Unexpected: ros.yaml is missing from the darknet config path " + cfg_path_config_folder)

        for f in self.cfg_files:
            yaml_stream = open(f, 'r')
            # Validate that it is a proper config file and gather weights file size info for load-time estimates
            cfg_dict = yaml.load(yaml_stream)
            #rospy.logwarn("" + str(cfg_dict))
            
            yaml_stream.close()
            if ("yolo_model" not in cfg_dict) or ("weight_file" not in cfg_dict["yolo_model"]) or ("name" not in cfg_dict["yolo_model"]["weight_file"]):
                rospy.logerr("Debug: " + str(cfg_dict))
                rospy.logwarn("File does not appear to be a valid A/I model config file: " + f + "... not adding this classifier")
                continue


            classifier_name = os.path.splitext(os.path.basename(f))[0]
            weight_file = os.path.join(self.model_search_path, "yolo_network_config", "weights",cfg_dict["yolo_model"]["weight_file"]["name"])
            if not os.path.exists(weight_file):
                rospy.logwarn("Classifier " + classifier_name + " specifies non-existent weights file " + weight_file + "... not adding this classifier")
                continue
            classifier_keys = list(cfg_dict.keys())
            classifier_key = classifier_keys[0]
            classifier_classes_list.append(cfg_dict[classifier_key]['detection_classes']['names'])
            #rospy.logwarn("classes: " + str(classifier_classes_list))
            classifier_name_list.append(classifier_name)
            classifier_size_list.append(os.path.getsize(weight_file))
        for i,name in enumerate(classifier_name_list):
            model_name = self.model_prefix + name
            model_dict = dict()
            model_dict['name'] = name
            model_dict['size'] = classifier_size_list[i]
            model_dict['classes'] = classifier_classes_list[i]
            models_dict[model_name] = model_dict
        #rospy.logwarn("Classifier returning models dict" + str(models_dict))
        return models_dict


    def startClassifier(self, classifier, input_img, threshold):
        # Build Darknet new classifier launch command
        launch_cmd_line = [
            "roslaunch", self.pkg_name, self.launch_file,
            "namespace:=" + self.node_namespace, 
            "yolo_weights_path:=" + os.path.join(self.model_search_path, "yolo_network_config/weights"),
            "yolo_config_path:=" + os.path.join(self.model_search_path, "yolo_network_config/cfg"),
            "ros_param_file:=" + os.path.join(self.model_search_path, "config/ros.yaml"),
            "network_param_file:=" + os.path.join(self.model_search_path, "config", classifier + ".yaml"),
            "input_img:=" + input_img,
            "detection_threshold:=" + str(threshold)
        ]
        rospy.loginfo("Launching Darknet ROS Process: " + str(launch_cmd_line))
        self.ros_process = subprocess.Popen(launch_cmd_line)
        

        # Setup Classifier Setup Tracking Progress




    def stopClassifier(self):
        rospy.loginfo("Stopping classifier")
        if not (None == self.ros_process):
            self.ros_process.terminate()
            self.ros_process = None
        self.current_classifier = "None"
        self.current_img_topic = "None"
        
        #self.current_threshold = 0.3

    def updateClassifierThreshold(self,threshold):
        set_threshold_pub.publish(theshold)


    

if __name__ == '__main__':
    DarknetAIF()
