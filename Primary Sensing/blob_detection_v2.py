import numpy as np
import cv2
import matplotlib as mpl
import os
import time

# A function to fix HSV range
def fixHSVRange(h, s, v):
    # Normal H,S,V: (0-360,0-100%,0-100%)
    # OpenCV H,S,V: (0-180,0-255 ,0-255)
    return (180 * h / 360, 255 * s / 100, 255 * v / 100)

def detectBlob(im,display_mask):
    # Make a copy of Image; find the HSV range; convert it to OpenCV
    # undrestandble range and make a mask from it
    frm=im.copy()
    frm = cv2.cvtColor(frm, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frm, fixHSVRange(0, 0, 0), fixHSVRange(360, 4.53, 100))

    # Remove the noise
    noise=cv2.dilate(mask,np.ones((5,5)))
    noise=cv2.erode(mask,np.ones((5,5)))
    noise=cv2.medianBlur(mask,7)

    # Change image channels
    mask=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    noise=cv2.cvtColor(noise,cv2.COLOR_GRAY2BGR)
    cleanMask=~noise

    # Make a new mask without noise
    centerMask=cv2.cvtColor(cleanMask.copy(),cv2.COLOR_BGR2GRAY)
    if display_mask:
        cv2.imshow("out",centerMask)
        cv2.waitKey(0)
    # Image detector
    keypoints = detector.detect(centerMask)

    centroids_x = np.array([])
    centroids_y = np.array([])
    for keypoint in keypoints:
        centroids_x = np.append(centroids_x, keypoint.pt[0])
        centroids_y = np.append(centroids_y, keypoint.pt[1])

    centroids_x = centroids_x.astype(int)
    centroids_y = centroids_y.astype(int)

    return zip(centroids_x,centroids_y)

def drawCentroids(image, centroids):
    im_with_centroids = image
    for centroid_x,centroid_y in centroids:
        im_with_centroids = cv2.circle(im_with_centroids, (centroid_x,centroid_y), radius=0, color=(0,0,255), thickness=20)
    return im_with_centroids
    
    

#Blob detector
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.filterByColor = False

# Change thresholds
params.minThreshold = 0
params.maxThreshold = 250


# Filter by Area.
params.filterByArea = True
params.minArea = 530
params.maxArea = 100000


# Filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.1
#params.maxCircularity = 0.8

# Filter by inertia ratio
params.filterByInertia = True
params.maxInertiaRatio = 0.94

# FIlter by convexity
params.filterByConvexity = False
params.minConvexity = 0.4


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)




mode = "vid"  #im or vid

if mode == "im":
    im = cv2.imread(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing\test_im8.png")
    H, W = im.shape[:2]
    centroid_coords = detectBlob(im,True)
    im_with_centroids = drawCentroids(im,centroid_coords)

    # Show blobs
    cv2.imshow("Centroids", im_with_centroids)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 


if mode == "vid":
    ## Video detector
    vid = cv2.VideoCapture(r"c:\Users\ykelm\Autonomous-AerialLocalizationTeam1\Primary Sensing\Full_test_vid.mp4")

    # Setup output video
    outpath = "Primary Sensing/Output Files/"
    size = (1920, 1080)    
    out = cv2.VideoWriter(outpath + "blob_detection_test2.MP4",cv2.VideoWriter_fourcc(*'MP4V'), 30, size)

    while(vid.isOpened()):
        print("here")
        _,frame = vid.read()
        if frame is None:
            break
        cnts = detectBlob(frame,False)
        im_with_centroids = drawCentroids(frame,cnts)
        cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("window",cv2.WND_PROP_FULLSCREEN,
               cv2.WINDOW_FULLSCREEN)
        cv2.imshow('window',im_with_centroids)

        #Write video
        out.write(im_with_centroids)

        key = cv2.waitKey(500)
        if key == 32:
            cv2.waitKey()
        if key == ord('q'): 
            break
    
    # After the loop release the vid objects
    vid.release() 
    out.release()
    # Destroy all the windows 
    cv2.destroyAllWindows()