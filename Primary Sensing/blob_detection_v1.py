import numpy as np
import cv2
import matplotlib as mpl
import os
import time


def detectBlob(image):
    start = time.time()

    color_low = (0,0,0)
    color_high = (255,255,255)
    mask = cv2.inRange(image,color_low,color_high)

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
     
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

params.filterByArea = False
params.filterByInertia = False
params.filterByConvexity = False

# Change thresholds

params.minThreshold = 0
params.maxThreshold = 255


# Filter by Area.
params.filterByArea = True
params.minArea = 500
params.maxArea = 100000


# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.05
#params.maxCircularity = 0.8


#Filter by Color
params.filterByColor = False
params.blobColor = 0


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

##### Image or video mode #####
mode = "im"

if mode == "im":
    # Read image
    im = cv2.imread(r"C:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing/mask4.png")
    centroid_coords = detectBlob(im)

    # Draw centroids of detected blobs
    im_with_centroids = drawCentroids(im,centroid_coords)

    # Show blobs
    cv2.imshow("Centroids", im_with_centroids)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

elif mode == "vid":
    ## Video detector
    vid = cv2.VideoCapture(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing\Full_test_vid")

    while(vid.isOpened()):
        _,frame = vid.read()
        if frame is None:
            break
        centroids = detectBlob(frame)
        im_with_centroids = drawCentroids(frame,centroids)
        im_with_centroids = cv2.resize(im_with_centroids, (640, 640))
        cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,
               cv2.WINDOW_FULLSCREEN)
        cv2.imshow('window',im_with_centroids)

        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
    
    # After the loop release the vid object 
    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows()











