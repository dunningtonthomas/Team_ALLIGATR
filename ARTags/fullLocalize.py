# Import libraries
import numpy as np
import matplotlib as mpl
import pandas as pd
import cv2
import pymap3d as pm
import csv


# ArUco dictionaries
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# %% Functions
# Highlight the detected markers
def aruco_display(corners, ids, rejected, currFrame, image):

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Define the center of the image
    cx = int(xPixels / 2)
    cy = int(yPixels / 2)

    # Draw Line from principle point forward along the drone x-axis
    cv2.arrowedLine(image, (cx, cy), (cx, cy - 100), (0, 255, 0), 2)


    # Draw the line facing north using the yaw angle, also draw the east direction in black
    if currFrame is not None:
        yaw = currFrame['yaw'] * np.pi / 180
        northAng = yaw * -1                         # Angle used to draw the north line given the yaw angle
        px = round(100 * np.sin(northAng))
        py = round(100 * np.cos(northAng)) 

        # North Axis
        cv2.arrowedLine(image, (cx, cy), (cx + px, cy - py), (0, 0, 0), 2)  
        
        # East direction
        eastAng = northAng + np.pi / 2
        px = round(100 * np.sin(eastAng))
        py = round(100 * np.cos(eastAng)) 

        # East Axis
        cv2.arrowedLine(image, (cx, cy), (cx + px, cy - py), (255, 255, 255), 2) 

    # Principle point
    cv2.circle(image, (cx, cy), radius=0, color=(255, 0, 0), thickness=10) 


    # Draw AR tag detection if the marker is detected
    if(len(corners) > 0):
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners  # Get the corners

            # Cast the data to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Centroid
            xf = round((topLeft[0] + bottomRight[0]) / 2)
            yf = round((topLeft[1] + bottomRight[1]) / 2)


            # Draw the lines for the AR tag detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
            cv2.circle(image, (xf,yf), radius=0, color=(0, 0, 255), thickness=10)


            # Draw line from principle point to the centroid              
            cv2.line(image, (cx, cy), (xf,yf), (255, 0, 0), 2)

            # Draw Rx and Ry lines in light blue color
            cv2.line(image, (cx, cy), (xf,cy), (220, 245, 150), 2)
            cv2.line(image, (xf,cy), (xf,yf), (220, 245, 150), 2)

    # Resize the image
    image = cv2.resize(image, (1280, 720))                # Resize image for display        
    return image


#### readSRT takes as input the full path to the SRT file and returns a list of dictionaries of the drone state
# The format of this state is a dictionary with the following format:
# {"lat" : lat, "long" : long, "alt" : alt, "yaw" : yaw, "pitch" : pitch, "roll" : roll}
# Each item in the list corresponds to a frame in the corresponding video
def readSRT(path):
    file = open(path, 'r')

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
        
        # Parse string by splitting the string up by certain delimeters
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

    # Close the file
    file.close()

    # Return the list of dictionaries containing the state parameters at each frame
    return outData


# rel_localize calculates the relative x and y coordinates in the drone frame from the principle point
def rel_localize(corners, height, ids):

    # Check if ids is empty
    if ids is None:
        return 0, 0

    # Pitch and roll are approximately zero
    theta = 0
    phi = 0

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Field of view, values based on the 84 diagonal fov of the MAVIC 3 camera
    HFOV = 76.25 * np.pi / 180 
    VFOV = 47.64 * np.pi / 180 

    # Focal length
    f = 24  # mm

    # Define the center of the image
    cx = xPixels / 2
    cy = yPixels / 2

    # Define scaling factors
    sx = f * np.tan(VFOV/2) / cx    # mm/pixel
    sy = f * np.tan(HFOV/2) / cy

    # Get the centroid coordinates of the AR tag
    firstCorners = corners[0][0]
    topLeft = firstCorners[0]
    bottomRight = firstCorners[2]

    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2

    # Relative camera coordinates in pixels
    yc = cy - yf
    xc = xf - cx

    # Calculate the angles
    alpha = np.arctan(xc * sx / f)
    beta = np.arctan(yc * sy / f)

    # Relative coordinates
    relX = height * np.tan(alpha + theta)
    relY = height * np.tan(beta + phi)

    return relX, relY

# AR_rel_localize calculates the relative x and y coordinates in the drone frame from the principle point
# This uses the known dimensions of the AR tag and their pixel lengths to calculate the conversion factor from pixels to meters on the ground
def AR_rel_localize(corners, height, ids):

    # Check if ids is empty
    if ids is None:
        return 0, 0

    # Pitch and roll are approximately zero
    theta = 0
    phi = 0

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Define the center of the image
    cx = xPixels / 2
    cy = yPixels / 2

    # Get the centroid coordinates of the AR tag and the corners
    firstCorners = corners[0][0]
    topLeft = firstCorners[1]
    topRight = firstCorners[2]
    bottomRight = firstCorners[3]
    bottomLeft = firstCorners[0]

    # Centroid
    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2

    # Length of each side of the AR Tag in pixels
    delL1 = np.sqrt((topLeft[0] - topRight[0])**2 + (topLeft[1] - topRight[1])**2)
    delL2 = np.sqrt((topRight[0] - bottomRight[0])**2 + (topRight[1] - bottomRight[1])**2)
    delL3 = np.sqrt((bottomRight[0] - bottomLeft[0])**2 + (bottomRight[1] - bottomLeft[1])**2)
    delL4 = np.sqrt((bottomLeft[0] - topLeft[0])**2 + (bottomLeft[1] - topLeft[1])**2)

    # Average length of the sides of the AR tag in pixels
    delL = (delL1 + delL2 + delL3 + delL4) / 4

    # Define AR Tag scaling factor
    s = 0.15875 / delL               # Meters per pixel

    # Relative camera coordinates in pixels
    yc = cy - yf
    xc = xf - cx

    # Calculate the angles using the distance to the RGV in the x and y axes and the height
    alpha = np.arctan(xc * s / height)
    beta = np.arctan(yc * s / height)

    # Relative coordinates in meters
    relX = height * np.tan(alpha + theta)
    relY = height * np.tan(beta + phi)

    return relX, relY

#### inertialCalc will calculate the inertial coordinates given the relative coordinates and state information
# relX and relY are the relative inerial coordinates of the target RGV
# currFrame is the current state of the drone including its GPS and state information
# startFrame is the first state of the drone to be used for the origin of the ENU frame
def inertialCalc(relX, relY, currFrame, startFrame):
    # Origin
    lat0 = startFrame["lat"]
    lon0 = startFrame["long"]
    h0 = 1600   # We don't care about the height

    # Drone current state
    lat = currFrame["lat"]
    lon = currFrame["long"]
    h = 1600   # We don't care about the height

    # Calculate the ENU coordinates in meters
    droneENU = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    Edrone = droneENU[0]
    Ndrone = droneENU[1]

    # Get the bearing angle from the yaw measurement
    p = currFrame['yaw'] * np.pi / 180

    # Rotate the relative measurements into the ENU frame
    Erel = np.cos(p) * relX + np.sin(p) * relY
    Nrel = -1*np.sin(p) * relX + np.cos(p) * relY

    # Calculate the RGV inertial position in the ENU frame
    ERGV = Edrone + Erel
    NRGV = Ndrone + Nrel

    return Edrone, Ndrone, ERGV, NRGV



# %% Main
# Dictionary
aruco_type = "DICT_6X6_250"
testDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()

# Read the Saved Video
path = "ARTags/Videos/"
videoPath = path + "Full_Localization.mp4"
cap = cv2.VideoCapture(videoPath)

# Define frame dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Read the SRT file with GPS and state data
srtPath = "ARTags/SRT_Files/Full_Localization.SRT"          
stateData = readSRT(srtPath)   # See the function definition for detailed framework of the stateData structure

# Assume constant height
height = 30

# Frame list for saving a video of output images
frames = []
i = 0                       # Iterator for the SRT data
startFrame = stateData[i]   # First frame of interest

# Begin to write to a file
# outputFilePath = "ARTags/OutputFiles/localization_1.csv"
# csv_file = open(outputFilePath, 'w', newline='')
# csv_writer = csv.writer(csv_file)
# csv_writer.writerow(['Frame', 'DroneE', 'DroneN', 'RGVE', 'RGVN'])  # Header


# Loop through video feed
while cap.isOpened():
    # Get the current video feed frame
    ret, img = cap.read()
    
	# Check if the image is empty
    if img is None:
        break

    # SRT data
    currState = stateData[i]    # State data for the current frame
    i = i + 1                   # Update iterator
    
    # Locate the Aruco tag
    corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)

    # Draw detection
    image = aruco_display(corners, ids, rejected, currState, img)
    frames.append(image)

	# Display the frame
    cv2.imshow('frame', image)

    # Check if the AR tag was detected, if so, calculate the position
    if ids is not None:
        # Current height 
        height = currState['alt']    # Relative altitude in meters

        # Calculate the relative x and y measurements in meters
        relX, relY = AR_rel_localize(corners, height, ids)

        # Calculate the inertial coordinates in the ENU frame in meters
        Edrone, Ndrone, ERGV, NRGV = inertialCalc(relX, relY, currState, startFrame)

        # Output the E and N coordinates of the detected RGV
        print("Drone E: ", Edrone, "\tRGV E: ", ERGV)
        print("Drone N: ", Ndrone, "\tRGV N: ", NRGV)
    else:
        # No RGV detected
        relX = 0
        relY = 0

        # Calculate the inertial coordinates in the ENU frame in meters
        Edrone, Ndrone, ERGV, NRGV = inertialCalc(relX, relY, currState, startFrame)
        ERGV = 0
        NRGV = 0    # 0 for not being detected

    # Save the E and N data
    #csv_writer.writerow([i, Edrone, Ndrone, ERGV, NRGV])

    # Wait until a key is pressed                      
    key = cv2.waitKey(0)
                
	# Quit
    cv2.waitKey(1)
    if key == ord("q"):
        break   


# Save images to a file
# outpath = "ARTags/OutputFiles/"
# size = (1920, 1080)    
# out = cv2.VideoWriter(outpath + "localization_1.MP4",cv2.VideoWriter_fourcc(*'MP4V'), 30, size)
#*'DIVX'

# Write to ouput video object
# for i in frames:
#     out.write(i)
# out.release()

# Close output text file
# csv_file.close()

# Close all and release
cv2.destroyAllWindows()
cap.release()

        






