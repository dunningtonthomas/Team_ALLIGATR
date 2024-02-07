#!/usr/bin/env python2 
# <- This is the shebang line which tells the OS which interpreter to use
# Be aware that the tutorial said to use python2, however we built it for py3 I believe

import rospy

if __name__ == '__main__': # <- Executable 
    rospy.init_node("blob_detection_node") # Initialize the ROS node

    rospy.loginfo("############# BLOB DETECTION NODE #################") # This will output to the terminal
    rospy.loginfo("Establishing camera connection...")

    camera_found = False
    while not camera_found:
        rospy.loginfo("Searching for camera...")

        # Implement code to check for camera connection here

        

        rospy.sleep(1.0) # Sleep for 1 second

        # If camera is not found, output an error message
        rospy.logerr("Camera not found. Retrying...")

    rospy.loginfo("End of program") # This will output to the terminal
