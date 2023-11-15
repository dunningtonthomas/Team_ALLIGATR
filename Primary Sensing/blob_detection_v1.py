import numpy as np
import cv2
import matplotlib as mpl
import os
import time


def detectBlob(image):
    start = time.time()
    # Detect blobs. Filter by color
    #result = image.copy()
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #lower = np.array([0,0,0])
    #upper = np.array([100,100,100])
    #mask = cv2.inRange(image, lower, upper)
    #result = cv2.bitwise_and(result, result, mask=mask)

    #cv2.imshow('mask', mask)
    #cv2.imshow('result', result)
    #cv2.waitKey()

    color_low = (0,0,0)
    color_high = (255,255,255)
    mask = cv2.inRange(image,color_low,color_high)
    #cv2.imshow('mask',mask)
    #cv2.waitKey(0)
    # Image detector
    keypoints = detector.detect(image)

    centroids_x = np.array([])
    centroids_y = np.array([])
    for keypoint in keypoints:
        centroids_x = np.append(centroids_x, keypoint.pt[0])
        centroids_y = np.append(centroids_y, keypoint.pt[1])

    centroids_x = centroids_x.astype(int)
    centroids_y = centroids_y.astype(int)

    time_elapsed = (time.time() - start)
    print(time_elapsed)
    return zip(centroids_x,centroids_y)

def drawCentroids(image, centroids):
    im_with_centroids = image
    for centroid_x,centroid_y in centroids:
        im_with_centroids = cv2.circle(im_with_centroids, (centroid_x,centroid_y), radius=0, color=(0,0,255), thickness=20)
    return im_with_centroids

# Read image
#im = cv2.imread(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing\square_in_field.png",1)
im = cv2.imread(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\ARTags\Markers\ExampleMarker.png",1)

# Read video feed
#vid = cv2.VideoCapture(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing\sample_vid.mp4",1)
         
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds

#params.minThreshold = 0
#params.maxThreshold = 255


# Filter by Area.
params.filterByArea = True
params.minArea = 1000
params.maxArea = 100000


# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.5
#params.maxCircularity = 0.8


#Filter by Color
params.filterByColor = True
params.blobColor = 1


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)


centroid_coords = detectBlob(im)

# Draw centroids of detected blobs
im_with_centroids = drawCentroids(im,centroid_coords)

# Show blobs
cv2.imshow("Centroids", im_with_centroids)
cv2.waitKey(0)
cv2.destroyAllWindows()


## Video detector
vid_path =  r'ARTags\Videos\AR_Tag_Test_Trim.mp4'
vid = cv2.VideoCapture(vid_path)

while(vid.isOpened()):
    _,frame = vid.read()
    if frame is None:
        break
    centroids = detectBlob(frame)
    im_with_centroids = drawCentroids(frame,centroids)
    cv2.imshow('frame',im_with_centroids)

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
  
# After the loop release the vid object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows()











