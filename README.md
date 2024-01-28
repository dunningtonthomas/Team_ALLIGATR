# Team ALLIGATR Project Code

Repository for Senior Projects files 2023-2024

# Engaging the drone:

1) Access the HUB directory with `cd HUB`

2) Run the HUB with `bash engage.sh main.cpp`

# Dependancies

-Cmake 3.15 (or later), https://medium.com/@rahulbagul330/how-to-install-cmake-on-ubuntu-18-04-linux-c585394226bf

-C++ compiler (Tested with GCC), https://code.visualstudio.com/docs/cpp/config-linux

-ROS Melodic, https://wiki.ros.org/melodic/Installation/Ubuntu
Notes: make sure to use `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3` on first make of catkin_ws since this repo. If you are using python for any of the packages, I have run into errors using python3 regardless of this setup. It is advisable to use python 2 whenever possible. If errors are encountered with running ros packages, make it is linked to the catkin_ws/src folder using `cd ~/catkin_ws/src` then `ln -s ~/Team_ALLIGATR/HUB/ros_packages`. 

-Ubuntu 18.04, 

-Xterm, Install with `sudo apt update` then `sudo apt-get install xterm`. This is used to open up new kernals that will run the ROS packages seperately.

# Accessing the Nano using VNC
SSH (Secure Shell) allows a user to open a terminal remotely that accesses the nano via wifi. VNC (Virtual Network Computing) allows a user to access the nano through the main desktop. The nano already comes preinstalled with ssh and a vnc viewer. There are two VNC modes with important differences: 1) Remote viewing duplicates the **current display** connected to the nano. This means that the local monitor must be on to use this mode. 2) Virtual viewing can be run headless and creates a virtual desktop on the remote computer.

**Connecting with SSH**
For the first time after turning on the nano on a new wifi, you need to find its IP address to connect to it. You can find it by calling `ifconfig` while plugged into a monitor. After you know its IP, you can connect to it remotely while on the same wifi using an SSH software like XMoba or Putty.

