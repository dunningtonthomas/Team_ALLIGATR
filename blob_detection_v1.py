import numpy as np
import cv2


# Read video feed

im = cv2.imread("square.jpg",1)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds

#params.minThreshold = 0
#params.maxThreshold = 255


# Filter by Area.
params.filterByArea = True
params.minArea = 10000
params.maxArea = 100000


# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.7
params.maxCircularity = 0.8


#Filter by Color
params.filterByColor = True
params.blobColor = 0


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)


# Detect blobs. Filter by color
color_low = (0,0,0)
color_high = (255,255,255)
mask = cv2.inRange(im,color_low,color_high)

keypoints = detector.detect(im)
centroids_x = np.array([])
centroids_y = np.array([])
for keypoint in keypoints:
    centroids_x = np.append(centroids_x, keypoint.pt[0])
    centroids_y = np.append(centroids_y, keypoint.pt[1])

centroids_x = centroids_x.astype(int)
centroids_y = centroids_y.astype(int)


# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob#
im_with_centroids = im
for centroid_x,centroid_y in zip(centroids_x,centroids_y):
    im_with_centroids = cv2.circle(im_with_centroids, (centroid_x,centroid_y), radius=0, color=(0,255,0), thickness=5)

# Show blobs
cv2.imshow("Centroids", im_with_centroids)
cv2.waitKey(0)









