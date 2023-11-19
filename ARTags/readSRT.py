# Import libraries
import numpy as np
import matplotlib as mpl
import pandas as pd
import cv2


# Read in the file
path = "ARTags/SRT_Files/"
spyPath = "SRT_Files/"
fullPath = spyPath + "Full_Localization.SRT"
file = open(fullPath, 'r')

# Read the file into a list with each line
allLines = file.readlines()

# Data structure framework of the relevant data
# Create a list of dictionaries with each dictionary representing a single time step
# Each dictionary will contain the lat, long, rel_alt, yaw, pitch, and roll
outData = []

# Parse through the lines
for line in allLines:
    if(line[0] != '['):  # Condition for line with relevant information
        continue    # Go to the next line
    
    # Parse string
    splitLine = line.split(": ")
    latTemp = splitLine[3].split(']')
    longTemp = splitLine[4].split(']')
    altTemp = splitLine[5].split(' ')
    yawTemp = splitLine[7].split(' ')
    pitchTemp = splitLine[8].split(' ')
    rollTemp = splitLine[9].split(']')
    
    # Save relevant data
    lat = float(latTemp[0])
    long = float(longTemp[0])
    alt = float(altTemp[0])
    yaw = float(yawTemp[0])
    pitch = float(pitchTemp[0])
    roll = float(rollTemp[0])
    
    # Create dictionary
    state = {"lat" : lat, "long" : long, "alt" : alt, "yaw" : yaw, "pitch" : pitch, "roll" : roll}
    
    # Append for total dictionary
    outData.append(state)


#Test 
currFrame = outData[0]
height = currFrame['alt'] * 3.28084


# Close the file
file.close()
















