#!/usr/bin/env python

import os
import os.path
import errno
import glob
import subprocess

import rospy

from std_msgs.msg import Empty
from num_sdk_msgs.srv import ImageClassifierListQuery, ImageClassifierListQueryResponse
from num_sdk_msgs.srv import ImageClassifierStatusQuery, ImageClassifierStatusQueryResponse
from num_sdk_msgs.msg import ClassifierSelection
from darknet_ros_msgs.msg import ObjectCount

class DarknetRosMgr:
    NODE_NAME = "DarknetRosMgr"
    DARKNET_CFG_PATH = '/opt/numurus/ros/share/num_darknet_ros/config/'

    darknet_cfg_files = []
    classifier_list = []

    current_classifier = "None"
    current_img_topic = "None"
    classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED

    darknet_ros_process = None

    found_object_sub = None

    def provide_classifier_list(self, req):
        return ImageClassifierListQueryResponse(self.classifier_list)

    def provide_classifier_status(self, req):
        return [self.current_img_topic, self.current_classifier, self.classifier_state]

    def stop_classifier(self):
        if not (None == self.darknet_ros_process):
            self.darknet_ros_process.terminate()
            self.darknet_ros_process = None
            self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_STOPPED

    def stop_classifier_cb(self, msg):
        self.stop_classifier()

    def start_classifier_cb(self, classifier_selection_msg):
        # First, validate the inputs
        # Check that the requested topic exists and has the expected type
        all_topics = rospy.get_published_topics()
        found_topic = False
        for t in all_topics:
            if (t[0] == classifier_selection_msg.img_topic) and (t[1] == 'sensor_msgs/Image'):
                found_topic = True
                break

        if (False == found_topic):
            rospy.logerr("Topic %s is not a valid image topic -- not starting classifier", classifier_selection_msg.img_topic)
            return

        # Check that the requested classifier exists
        if not (classifier_selection_msg.classifier in self.classifier_list):
            rospy.logerr("Unknown classifier requested: %s", classifier_selection_msg.classifier)
            return

        classifier_cfg_file = self.DARKNET_CFG_PATH + classifier_selection_msg.classifier + ".yaml"

        # Validate the requested_detection threshold
        if (classifier_selection_msg.detection_threshold < 0.0 or classifier_selection_msg.detection_threshold > 1.0):
            rospy.logerr("Requested detection threshold out of range (0.0 - 1.0)")
            return

        # Stop the current classifier if it is running
        self.stop_classifier()

        # Build up the arg string for the new classifier launch command
        namespace_arg = "namespace:=" + rospy.get_namespace();
        network_param_file_arg = "network_param_file:=" + classifier_cfg_file
        input_img_arg = "input_img:=" + classifier_selection_msg.img_topic
        detection_threshold_arg = "detection_threshold:=" + str(classifier_selection_msg.detection_threshold)

        rospy.loginfo("Launching Darknet ROS Process: %s, %s, %s, %s", namespace_arg, network_param_file_arg, input_img_arg, detection_threshold_arg)
        self.darknet_ros_process = subprocess.Popen(["roslaunch", "num_darknet_ros", "darknet_ros.launch", namespace_arg, network_param_file_arg, input_img_arg, detection_threshold_arg])

        # Update our local status
        self.current_classifier = classifier_selection_msg.classifier
        self.current_img_topic = classifier_selection_msg.img_topic
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_LOADING

        # Resubscribe to found_object so that we know when the classifier is up and running again
        self.found_object_sub = rospy.Subscriber('classifier/found_object', ObjectCount, self.got_darknet_update)

    def got_darknet_update(self, msg):
        # Means that darknet is up and running
        self.classifier_state = ImageClassifierStatusQueryResponse.CLASSIFIER_STATE_RUNNING
        self.found_object_sub.unregister()

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

        rospy.spin()

if __name__ == '__main__':
	DarknetMgr = DarknetRosMgr()
