#!/bin/bash

success () {
	tput setaf 2
	echo "--- $1 ----\n"
	tput sgr0
}

error () {
	tput setaf 1
	echo "--- $1 ----\n"	
	tput sgr0
}

success "Building haywyre arduino nodes"
# change this to your arduion library path.
export EXT_LIB_DIR=/home/$USER/sketchbook/libraries/
# run catkin on arduino firmware package
# build an image for each port.
for i in `ls /dev | grep ttyACM`; do
	echo $i ;
	# will be read by cmake when catkin_make runs.
	export CURRENT_ARDUINO_PORT=$i 
	success "\t\tBuilding for $CURRENT_ARDUINO_PORT"
	catkin_make
	if [ $? -eq 0 ]; then
    	success "Successfully built for $CURRENT_ARDUINO_PORT"
	else
	    error "Failed to build for $CURRENT_ARDUINO_PORT"
	    break
	fi
	#catkin_make haywyre_arduino_firmware_hello-upload
	catkin_make control_firmware_src_hello-upload
	if [ $? -eq 0 ]; then
    	success "Successfully uploaded firmware for $CURRENT_ARDUINO_PORT"
	else
	    error "Failed to uploaded firmware for $CURRENT_ARDUINO_PORT"
	    break
	fi
done;

bash ./run.sh &
sleep 2
pkill -P $$


success "Building haywyre control node"

catkin_make -DCATKIN_BLACKLIST_PACKAGES="haywyre_vision"
#catkin_make --pkg haywyre_control
# catkin_make --pkg haywyre_vision

if [ $? -eq 0 ]; then
	success "Successfully built for control node"
else
    error "Failed to build control node."
fi

bash ./run.sh &
sleep 2
pkill -P $$







