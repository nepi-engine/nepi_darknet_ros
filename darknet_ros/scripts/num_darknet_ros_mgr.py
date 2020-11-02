#!/usr/bin/env python

import os
import os.path
import errno
import glob
import subprocess

import rospy

from std_msgs.msg import Empty, Float32
from num_sdk_msgs.srv import ImageClassifierListQuery, ImageClassifierListQueryResponse
from num_sdk_msgs.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse
from num_sdk_msgs.msg import ClassifierSelection
from darknet_ros_msgs.msg import ObjectCount
from darknet_ros_msgs.msg import BoundingBoxes

from num_sdk_base.save_cfg_if import SaveCfgIF
from num_sdk_base.save_data_if import SaveDataIF

class DarknetRosMgr:
    NODE_NAME = "DarknetRosMgr"
    DARKNET_CFG_PATH = '/opt/numurus/ros/share/num_darknet_ros/config/'
    MIN_THRESHOLD = 0.001
    MAX_THRESHOLD = 1.0

    darknet_cfg_files = []
    classifier_list = []

    current_classifier = "None"
    current_img_topic = "None"
    current_threshold = 0.3
    classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED

    darknet_ros_process = None

    found_object_sub = None

    def provide_classifier_list(self, req):
        return ImageClassifierListQueryResponse(self.classifier_list)

    def provide_classifier_status(self, req):
        return [self.current_img_topic, self.current_classifier, self.classifier_state, self.current_threshold]

    def stop_classifier(self):
        if not (None == self.darknet_ros_process):
            self.darknet_ros_process.terminate()
            self.darknet_ros_process = None
            self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED
            self.current_classifier = "None"
            self.current_img_topic = "None"
            #self.current_threshold = 0.0

    def stop_classifier_cb(self, msg):
        self.stop_classifier()

    def start_classifier_cb(self, classifier_selection_msg):
        self.start_classifier(classifier=classifier_selection_msg.classifier, input_img=classifier_selection_msg.img_topic, threshold=classifier_selection_msg.detection_threshold)

    def set_threshold_cb(self, msg):
        # All we do here is update the current_threshold so that it is up-to-date in status responses
        # and will be saved properly in the config file (on request).
        if (msg.data >= self.MIN_THRESHOLD and msg.data <= self.MAX_THRESHOLD):
            self.current_threshold = msg.data

    def start_classifier(self, classifier, input_img, threshold):
        # First, validate the inputs
        # Check that the requested topic exists and has the expected type
        all_topics = rospy.get_published_topics()
        found_topic = False
        for t in all_topics:
            if (t[0] == input_img) and (t[1] == 'sensor_msgs/Image'):
                found_topic = True
                break

        if (False == found_topic):
            rospy.logerr("Topic %s is not a valid image topic -- not starting classifier", input_img)
            return

        # Check that the requested classifier exists
        if not (classifier in self.classifier_list):
            rospy.logerr("Unknown classifier requested: %s", classifier)
            return

        classifier_cfg_file = self.DARKNET_CFG_PATH + classifier + ".yaml"

        # Validate the requested_detection threshold
        if (threshold < self.MIN_THRESHOLD or threshold > self.MAX_THRESHOLD):
            rospy.logerr("Requested detection threshold (%f) out of range (0.001 - 1.0)", threshold)
            return

        # Stop the current classifier if it is running
        self.stop_classifier()

        # Build up the arg string for the new classifier launch command
        namespace_arg = "namespace:=" + rospy.get_namespace();
        network_param_file_arg = "network_param_file:=" + classifier_cfg_file
        input_img_arg = "input_img:=" + input_img
        detection_threshold_arg = "detection_threshold:=" + str(threshold)

        rospy.loginfo("Launching Darknet ROS Process: %s, %s, %s, %s", namespace_arg, network_param_file_arg, input_img_arg, detection_threshold_arg)
        self.darknet_ros_process = subprocess.Popen(["roslaunch", "num_darknet_ros", "darknet_ros.launch", namespace_arg, network_param_file_arg, input_img_arg, detection_threshold_arg])

        # Update our local status
        self.current_classifier = classifier
        self.current_img_topic = input_img
        self.current_threshold = threshold
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING

        # Resubscribe to found_object so that we know when the classifier is up and running again
        self.found_object_sub = rospy.Subscriber('classifier/found_object', ObjectCount, self.got_darknet_update)

    def got_darknet_update(self, msg):
        # Means that darknet is up and running
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING
        self.found_object_sub.unregister()

    def setCurrentSettingsAsDefault(self):
        rospy.set_param('~default_classifier', self.current_classifier)

        rospy.set_param('~default_image', self.current_img_topic)

        rospy.set_param('~default_threshold', self.current_threshold)

    def boundingBoxesCallback(self, msg):
        # Nothing to do if it isn't time to save yet.
        if not self.save_data_if.data_product_should_save('detection_bounding_boxes'):
            return

        full_path_filename = self.save_data_if.get_filename_path_and_prefix() + 'classifier_bounding_boxes_' + self.save_data_if.get_timestamp_string(msg.header.stamp) + '.txt'

        with open(full_path_filename, 'w') as f:
            f.write('input_image: ' + self.current_img_topic + '\n')
            f.write(str(msg))

    def updateFromParamServer(self):
        try:
            default_classifier = rospy.get_param('~default_classifier')
            default_img_topic = rospy.get_param('~default_image')
            default_threshold = rospy.get_param('~default_threshold')
            if default_classifier != "None" and default_img_topic != "None":
                rospy.loginfo("Classifier sleeping for 5 seconds to allow cameras to start")
                rospy.sleep(5) # Let the cameras start up properly
                rospy.loginfo('Starting classifier with parameters [' + default_classifier + ', ' + default_img_topic + ', ' + str(default_threshold) + ']')
                self.start_classifier(default_classifier, default_img_topic, default_threshold)
        except KeyError:
            rospy.loginfo("Classifier unable to find default parameters... starting up with no classifier running")

    def __init__(self):
        rospy.loginfo("Starting " + self.NODE_NAME + " Node")
        rospy.init_node(self.NODE_NAME)

        # Grab the list of all existing darknet cfg files
        self.darknet_cfg_files = glob.glob(self.DARKNET_CFG_PATH + '*.yaml')
        # Remove the ros.yaml file -- that one doesn't represent a selectable trained neural net
        try:
            self.darknet_cfg_files.remove(self.DARKNET_CFG_PATH + "ros.yaml")
        except:
            rospy.logwarn("Unexpected: ros.yaml is missing from the darknet config path %s", self.DARKNET_CFG_PATH)

        for f in self.darknet_cfg_files:
            self.classifier_list.append(os.path.splitext(os.path.basename(f))[0])

        if (len(self.classifier_list) < 1):
            rospy.logwarn("No classiers identified for this system at %s", self.DARKNET_CFG_PATH)

        rospy.Service('img_classifier_list_query', ImageClassifierListQuery, self.provide_classifier_list)
        rospy.Service('img_classifier_status_query', ImageClassifierStatusQuery, self.provide_classifier_status)

        rospy.Subscriber('start_classifier', ClassifierSelection, self.start_classifier_cb)
        rospy.Subscriber('stop_classifier', Empty, self.stop_classifier_cb)
        rospy.Subscriber('num_darknet_ros/set_threshold', Float32, self.set_threshold_cb)

        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        # Load default params
        #namespace = rospy.get_namespace()
        self.updateFromParamServer()

        # Setup for data saving -- just bounding boxes for now
        self.save_data_if = SaveDataIF(['detection_bounding_boxes'])
        rospy.Subscriber('classifier/bounding_boxes', BoundingBoxes, self.boundingBoxesCallback)

        rospy.spin()

if __name__ == '__main__':
	DarknetMgr = DarknetRosMgr()
