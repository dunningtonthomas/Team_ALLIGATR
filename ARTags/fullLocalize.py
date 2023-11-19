# Import libraries
import numpy as np
import matplotlib as mpl
import pandas as pd
import cv2
import pymap3d as pm


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

    # Principle point
    cv2.circle(image, (cx, cy), radius=0, color=(255, 0, 0), thickness=10) 

    # Draw Line from principle point forward along the drone x-axis
    cv2.line(image, (cx, cy), (cx, cy - 100), (0, 255, 0), 2)


    # Draw the line facing north using the yaw angle
    if currFrame is not None:
        yaw = currFrame['yaw'] * np.pi / 180
        px = round(100 * np.cos(yaw))
        py = round(100 * np.sin(yaw)) 

        # North Axis
        cv2.line(image, (cx, cy), (cx + px, cy - py), (255, 255, 255), 2)  

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

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Field of view, values based on the 84 diagonal fov of the MAVIC 3 camera
    HFOV = 76.25 * np.pi / 180 
    VFOV = 47.64 * np.pi / 180 

    # Define the center of the image
    cx = xPixels / 2
    cy = yPixels / 2

    # Get the centroid coordinates of the AR tag
    firstCorners = corners[0][0]
    topLeft = firstCorners[0]
    bottomRight = firstCorners[2]

    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2

    # Half of the image frame projected on the ground, distance in meters
    dx = height * np.tan(HFOV / 2)
    dy = height * np.tan(VFOV / 2)

    # Scaling factors, conversion in pixels per meter
    sx = cx / dx
    sy = cy / dy

    # Calcaulte the depth of the principal point projection on the ground, assume camera is pointed directly down
    Zp = height    

    # Relative coordinates
    relX = (xf - cx) / sx
    relY = (cy - yf) / sy

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

    # Convert the relative measurements to meters
    relX_m = relX * 0.3048
    relY_m = relY * 0.3048

    # Get the bearing angle from the yaw measurement
    p = currFrame['yaw'] * np.pi / 180

    # Rotate the relative measurements into the ENU frame
    Erel = np.cos(p) * relY_m + np.sin(p) * relX_m
    Nrel = -1*np.sin(p) * relY_m + np.cos(p) * relX_m

    # Calculate the RGV inertial position in the ENU frame
    Ecoord = Edrone + Erel
    Ncoord = Ndrone + Nrel

    return Ecoord, Ncoord


# %% Main
# Dictionary
aruco_type = "DICT_6X6_250"
testDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()

# Read the Saved Video
path = "ARTags/Videos/"
videoPath = path + "Full_Localization.mp4"
cap = cv2.VideoCapture(videoPath)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Read the SRT file with GPS and state data
srtPath = "ARTags/SRT_Files/Full_Localization.SRT"
stateData = readSRT(srtPath)   # See the function definition for detailed framework of the stateData structure

# Assume constant height
height = 30

# Frame list
frames = []

i = 0                       # Iterator for the SRT data
startFrame = stateData[i]   # First frame of interest
while cap.isOpened():
    # Get the current video feed frame
    ret, img = cap.read()
    
	# Check if the image is empty
    if img is None:
        break

    # SRT data
    currFrame = stateData[i]    # State data for the current frame
    i = i + 1                   # Update iterator
    
    # Locate the Aruco tag
    corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)

    # Draw detection
    image = aruco_display(corners, ids, rejected, currFrame, img)
    frames.append(image)

	# Display the frame
    cv2.imshow('frame', image)

    # Check if the AR tag was detected, if so, calculate the position
    if ids is not None:
        # Current height 
        height = currFrame['alt'] * 3.28084     # Relative altitude in feet

        # Calculate the relative x and y measurements
        relX, relY = rel_localize(corners, height, ids)

        # Calculate the inertial coordinates in the ENU frame
        Ecoord, Ncoord = inertialCalc(relX, relY, currFrame, startFrame)
        print("E: ", Ecoord)
        print("N: ", Ncoord)

        # Output relative x and y measurements
        # print("Relative X:\t", relX)
        # print("Relative Y:\t", relY)
        # print("Altitude: ", height)


    # Wait until a key is pressed            
    cv2.waitKey(0)
                
	# Quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break



# Save images to a file
# size = (1920, 1080)    
# out = cv2.VideoWriter(path + "arDetect.mp4",cv2.VideoWriter_fourcc(*'DIVX'), 30, size)

# Write to ouput video object
# for i in frames:
#     out.write(i)
# out.release()


cv2.destroyAllWindows()
cap.release()	







