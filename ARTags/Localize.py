# Import libraries
import numpy as np
import matplotlib as mpl
import pandas as pd
import cv2

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
def aruco_display(corners, ids, rejected, image):
    if(len(corners) > 0): # Are any aruco tags detected
        
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

            # Image is 1920 by 1080 pixels
            xPixels = 1920
            yPixels = 1080

            # Define the center of the image
            cx = int(xPixels / 2)
            cy = int(yPixels / 2)

            # Draw the lines for the AR tag detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
            cv2.circle(image, (xf,yf), radius=0, color=(0, 0, 255), thickness=10)

            # Draw principle point and line from principle point to the centroid
            cv2.circle(image, (cx, cy), radius=0, color=(0, 255, 0), thickness=10)
            cv2.line(image, (cx, cy), (xf,yf), (0, 255, 0), 2)

            # Draw Line from principle point forward along the drone x-axis
            cv2.line(image, (cx, cy), (cx, cy - 100), (0, 255, 0), 2)
            
            # Resize the image
            image = cv2.resize(image, (1280, 720))                # Resize image for display
            

    return image

def localize(corners, height, ids):

    # Check if ids is empty
    if ids is None:
        return 0, 0

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Field of view, standard values for our wide fov camera
    HFOV = 100 * np.pi / 180 
    VFOV = 70 * np.pi / 180 

    # Define the center of the image
    cx = xPixels / 2
    cy = yPixels / 2

    # Define the focal length and principal point translation, this is the intrinsic camera matrix
    f = 2.87

    # Get the centroid coordinates of the AR tag
    firstCorners = corners[0][0]
    topLeft = firstCorners[0]
    bottomRight = firstCorners[2]

    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2


    # Using the center, find the relative position of the RGV
    # Compute the 2D normalized coordinates with respect to the intrinsic camera matrix
    dx = f * np.tan(HFOV / 2)
    dy = f * np.tan(VFOV / 2)

    # Scaling factors
    sx = cx / dx
    sy = cy / dy

    # Calcaulte the depth of the principal point projection on the ground
    Zp = height    

    # Distance calculation
    d = np.sqrt(((cx - xf) / sx)**2 + ((cy - yf) / sy)**2)
    theta = np.arctan(d / f)
    Z = Zp / np.cos(theta)

    # Bearing calculation
    p = np.arctan2((xf - cx), (cy - yf))   

    # Range and bearing output
    range = Z
    bearing = p * 180 / np.pi   # Deg

    # Return the range and bearing measurements
    return range, bearing



# Localize 2 doesn't use focal length but the height of the drone
def localize2(corners, height, ids):

    # Check if ids is empty
    if ids is None:
        return 0, 0

    # Image is 1920 by 1080 pixels
    xPixels = 1920
    yPixels = 1080

    # Field of view, standard values for our wide fov camera
    HFOV = 100 * np.pi / 180 
    VFOV = 70 * np.pi / 180 

    # Define the center of the image
    cx = xPixels / 2
    cy = yPixels / 2

    # Get the centroid coordinates of the AR tag
    firstCorners = corners[0][0]
    topLeft = firstCorners[0]
    bottomRight = firstCorners[2]

    xf = (topLeft[0] + bottomRight[0]) / 2
    yf = (topLeft[1] + bottomRight[1]) / 2


    # Using the center, find the relative position of the RGV
    # Compute the 2D normalized coordinates with respect to the intrinsic camera matrix
    dx = height * np.tan(HFOV / 2)
    dy = height * np.tan(VFOV / 2)

    # Scaling factors
    sx = cx / dx
    sy = cy / dy

    # Calcaulte the depth of the principal point projection on the ground
    Zp = height    

    # Distance calculation
    d = np.sqrt(((cx - xf) / sx)**2 + ((cy - yf) / sy)**2)
    theta = np.arctan(d / height)
    Z = Zp / np.cos(theta)

    # Bearing calculation
    p = np.arctan2((xf - cx), (cy - yf))   

    # Range and bearing output
    range = Z
    bearing = p * 180 / np.pi   # Deg

    # Return the range and bearing measurements
    return range, bearing



# %% Main
# Dictionary
aruco_type = "DICT_6X6_250"
testDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters()


# %% Video Capture With Webcam
path = "ARTags/Videos/"
videoPath = path + "AR_Tag_Test_Trim.mp4"
cap = cv2.VideoCapture(videoPath)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Assume constant height
height = 30

while cap.isOpened():
    # Get the current video feed frame
    ret, img = cap.read()
    
	# Check if the image is empty
    if img is None:
        break
    
    # Locate the Aruco tag
    corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)
    image = aruco_display(corners, ids, rejected, img)
    
	# Display the frame
    cv2.imshow('frame', image)

    # Output the range and bearing
    range, bearing = localize2(corners, height, ids)

    # Print results
    if range == 0 and bearing == 0:
        continue

    print("Range: ", range)
    print("Bearing: ", bearing)

    # Wait until a key is pressed
    cv2.waitKey(0)
    
	# Quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    
	
cv2.destroyAllWindows()
cap.release()	







