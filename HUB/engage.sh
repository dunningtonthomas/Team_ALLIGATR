#!/bin/bash

# Save the file name as a variable called SOURCE_FILE
SOURCE_FILE=$1

# Function to kill all child processes when script exits
function cleanup {
    pkill -P $$
}
trap cleanup EXIT

# This will clear any previous builds before building again
rm build -rf
mkdir -p build
cd build

#Compile the ROS packages
CURRENT_DIR=$(pwd) #Save the current directory as a variable
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd ${CURRENT_DIR} #Go back to the build directory

#Opens a roscore terminal. If one already exists, it will close itself
xterm -e "source ~./bashrc; roscore; exit; exec bash" &

# Launch the XTERM terminal and run the ROS node
# Make copies of this line of code for any additional nodes
xterm -e "source ~./bashrc; rosrun ros_beginner_package first_node.py; exec bash" &

#Make the project using the passed in source file
cmake -DSOURCE_FILE=${SOURCE_FILE} ..
make
./main
cd ..