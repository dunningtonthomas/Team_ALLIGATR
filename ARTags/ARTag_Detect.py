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

            # Draw the lines for the AR tag detection
            cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)

    return image



# Path for aruco tags
path = "Autonomous-AerialLocalizationTeam1/ARTags/Markers/"

# Get predefined dictionary
aruco_type = "DICT_6X6_250"
testDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

# Insantiate parameters
arucoParams = cv2.aruco.DetectorParameters()

# Read in the image
imagePath = path + "ExampleMarker.png"
img = cv2.imread(imagePath, cv2.IMREAD_COLOR)

# Display the read in image
cv2.imshow("ArUco Marker", img)

# Wait until a key is pressed
cv2.waitKey(0)

# Close all of the windows
cv2.destroyAllWindows()

  
# Detect the marker
corners, ids, rejected = cv2.aruco.detectMarkers(img, testDict, parameters=arucoParams)

# Draw the detection
image = aruco_display(corners, ids, rejected, img)

# Display the image with the detected AR tag
cv2.imshow("ArUco Marker", image)

# Image is 1280 by 720 pixels
xPixels = 1280
yPixels = 720

# Field of view
FOV = 100 * np.pi / 180 # (RAD)

# Define the center of the image
cx = xPixels / 2
cy = yPixels / 2

# Define the focal length and principal point translation, this is the intrinsic camera matrix
f = 0.01	# Focal length in m
fx = 10	
fy = 10

# Define the height and angle of the camera
H = 30
alpha = 0

# Get the centroid coordinates of the AR tag
firstCorners = corners[0][0]
topLeft = firstCorners[0]
bottomRight = firstCorners[2]

xf = (topLeft[0] + bottomRight[0]) / 2
yf = (topLeft[1] + bottomRight[1]) / 2

print("Center of AR Tag (Pixels): ", xf, " ", yf)

# %% Using the center, find the relative position of the RGV

# Compute the 2D normalized coordinates with respect to the intrinsic camera matrix
xn = (xf - cx) / fx
yn = (yf - cy) / fx

# Calcaulte the depth of the principal point projection on the ground
Zp = H / np.cos(alpha)

# Angles from the field of view
vertAng = FOV / 2
horzAng = FOV / 2

# Number of pixels
dx = f * 1000 * np.tan(horzAng)
dy = f * 1000 * np.tan(vertAng)

# Scaling factors
sx = cx / dx
sy = cy / dy

# Distance calculation
d = np.sqrt(((cx - xf) / sx)**2 + ((cy - yf) / sy)**2)
theta = np.arctan(d / f)
Z = Zp / np.cos(theta)

# Camera frame coordinates
xc = xn * Z
yc = yn * Z
zc = Z

# Print results
print("Camera frame coordinates: ", "X: ", xc, "Y: ", yc, "Z: ", zc)

# Wait until a key is pressed
cv2.waitKey(0)

# Close all of the windows
cv2.destroyAllWindows()




