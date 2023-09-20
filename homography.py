import numpy as np
import cv2
from matplotlib import pyplot as plt
from math import cos,sin
import glob
import io
import time
from time import sleep

#hard coded camera intrinsic matrices
cameraMatrix = [[2.40140558e+03,0.00000000e+00,1.04908260e+03],[0.00000000e+00,2.37984043e+03,9.84406691e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]]
dist = [[ 0.03913461,0.35405608,0.00330635,-0.03732458,-0.9110321 ]]

simulate = True
mappingUndistort = True
img = []
dst = []
# Load our image
if simulate == True:
    im = cv2.cvtColor(cv2.imread('Drone_1.jpg'), cv2.COLOR_BGR2GRAY)

    # Rotate 27 degrees, translate -15,5 pixels (simulation)
    th = 27*np.pi/180
    tx=-15
    ty =5
    M = np.array([[np.cos(th),-np.sin(th),tx],
                 [np.sin(th),np.cos(th),ty],
                  [0,0,1]])
    # Simulate the change in view that would we would undergo given this transformation
    im_new = cv2.warpPerspective(im,M,im.shape,borderMode=cv2.BORDER_REPLICATE)
else:
    cam = cv2.VideoCapture(0)

    previous=time.time()
    delta=0

    while True:
        current = time.time()
        delta += current-previous
        previous = current

        _,img = cam.read()
        if delta > 1:
            h, w = img.shape[:2]
            newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))
            if mappingUndistort == True:
                mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w, h), 5)
                dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
                x, y, w, h = roi
                dst = dst[y:y + h, x:x + w]
            else:
                dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)
                x, y, w, h = roi
                dst = dst[y:y + h, x:x + w]

# Initiate ORB detector
orb = cv2.ORB_create()

# Find the keypoints and descriptors
kp1, des1 = orb.detectAndCompute(img,None)
kp2, des2 = orb.detectAndCompute(dst,None)

# Match keypoints
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1,des2)

# Sort matches
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 20 matches.
plt.figure(figsize=(15,5))
img3 = cv2.drawMatches(im,kp1,im_new,kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(img3)
plt.show()

# Find keypoint correspondences
X1 = np.vstack([kp1[match.queryIdx].pt for match in matches])
X2 = np.vstack([kp2[match.trainIdx].pt for match in matches])

# Estimate homograpahy using opencv -
H, mask = cv2.findHomography(X1, X2, cv2.RANSAC, 1.0)

# Let's apply the homography to our image - in theory it should generate the second picture
im_warp = cv2.warpPerspective(im,H,(im.shape[0],im.shape[1]))

plt.figure(figsize=(15,5))
plt.subplot(1,3,1)
plt.imshow(im)
plt.title('Image 1')
plt.subplot(1,3,2)
plt.imshow(im_warp)
plt.title('Image 1 warped by Homography')
plt.subplot(1,3,3)
plt.imshow(im_new)
plt.title('Image 2')
plt.show()

print('H',H)
print('Map', M)
print ('The angle of rotation is %2.2f degrees.'%(np.arctan2(H[0,1],H[0,0])*180/np.pi))
print ('The translation is x=%2.2f, y=%2.2f pixels'%(H[0,2],H[1,2]))