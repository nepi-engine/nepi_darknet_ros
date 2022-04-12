#!/bin/bash

# Can set the environment variables in a separate file sourced in (e.g., ~/bash.rc),
# set them here by uncommenting, or do it interactively (with defaults)
# by leaving these commented out
#SSH_KEY_PATH=$HOME/.ssh/numurus_3dx_jetson_sshkey
#REMOTE_HOST=192.168.179.102
#SRC_PATH=`pwd`

if [ -z "$SSH_KEY_PATH" ] || [ ! -f "$SSH_KEY_PATH" ]; then
	read -e -p "Enter the path to the SSH key for the remote system
	> " -i $HOME/.ssh/numurus_3dx_jetson_sshkey SSH_KEY_PATH
	if [ ! -f "$SSH_KEY_PATH" ]; then
		echo "Error: "$SSH_KEY_PATH" does not exist... exiting"
		exit 1
	fi
fi

if [ -z "$REMOTE_HOST" ]; then
	REMOTE_HOST=192.168.179.102
	read -e -p "Enter the remote hostname or IP address
	> " -i $REMOTE_HOST REMOTE_HOST
fi
   
# Set the source path for classifier
if [ -z "$SRC_PATH" ] || [ ! -d $SRC_PATH ]; then
	read -e -p "Enter the path to the deployment folder (parent of config and yolo_network_config directories)
	> " -i `pwd` SRC_PATH
	if [ ! -d "$SRC_PATH/config" ] || [ ! -d "$SRC_PATH/yolo_network_config/cfg" ] || \
	   [ ! -d "$SRC_PATH/yolo_network_config/weights" ]; then
		echo "Error: $SRC_PATH does not have proper subdirectories... exiting"
		exit 1
	fi
fi

#cd $SRC_PATH
classifier_list=($SRC_PATH/config/*.yaml)
#echo "Found classifiers: ${classifier_list[@]}"

for classifier_cfg in "${classifier_list[@]}"; do
	classifier_yaml=${classifier_cfg/"$SRC_PATH/config/"/}
	classifier_name=${classifier_yaml/".yaml"/}
	#echo $classifier_name
	read -e -p "Do you want to deploy $classifier_name (Y/N)?
	> " -i "Y" answer
	if [ $answer = "Y" ]; then
		classifier_net_cfg="$SRC_PATH/yolo_network_config/cfg/$classifier_name.cfg"
		if [ ! -f $classifier_net_cfg ]; then
			echo "Error: $classifier_net_cfg does not exist"
			exit 1
		fi
		classifier_weights_file="$SRC_PATH/yolo_network_config/weights/$classifier_name.weights"
		if [ ! -f $classifier_weights_file ]; then
			echo "Error: $classifier_weights_file does not exist"
			exit 1
		fi
		printf "Deploying classifier $classifier_name with source files\n\n"
		printf "   $classifier_cfg\n\n"
		printf "   $classifier_net_cfg\n\n"
		printf "   $classifier_weights_file\n\n"

		DARKNET_DIR=/opt/numurus/ros/share/num_darknet_ros
		#echo "scp -i $SSH_KEY_PATH $classifier_cfg numurus@$REMOTE_HOST:$DARKNET_DIR/config/"
		scp -i $SSH_KEY_PATH $classifier_cfg numurus@$REMOTE_HOST:$DARKNET_DIR/config/
		scp -i $SSH_KEY_PATH $classifier_net_cfg numurus@$REMOTE_HOST:$DARKNET_DIR/yolo_network_config/cfg/
		scp -i $SSH_KEY_PATH $classifier_weights_file numurus@$REMOTE_HOST:$DARKNET_DIR/yolo_network_config/weights/
	else
		echo "Skipping $classifier_name"
	fi
done
