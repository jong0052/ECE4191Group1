import numpy as np
from matplotlib import pyplot as plt
import cv2
from math import cos, sin

#test case
fig = plt.figure(figsize = (10, 7))
ax = plt.axes(projection ="3d")

x = np.linspace(-0.1,0.1,100)
y = 0.1*np.sin(50*x)
z = np.ones(100)

ax.scatter3D(x,y,z)
ax.plot([0,0.05],[0,0],[0,0],'r')
ax.plot([0,0],[0,0.05],[0,0],'g')
ax.plot([0,0],[0,0],[0,0.05],'b')
plt.xlim(-0.5,0.5)
plt.ylim(-0.5,0.5)
ax.set_zlim(0,1)
plt.show()

K = np.array([[1200,0,340],[0,1200,240],[0,0,1]])

# Create a coordinate vector for all points
X = np.vstack([x,y,z])

print("We now have 100 different coordinate vectors:",X.shape)

# Project into image plane
im_coords_scaled = K@X
im_coords = im_coords_scaled/im_coords_scaled[2,:]

plt.plot(im_coords[0,:],im_coords[1,:],'o')
plt.xlim(0,640)
plt.ylim(0,480)
plt.show()

K = np.array([[1200,0,340],[0,1200,240],[0,0,1]])


def get_transformation_matrix(roll,pitch,yaw,tx,ty,tz):
    return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), tx],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), ty],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), tz]
             ])

# Rotate camera 45 degrees and pull it back 0.5 m
T = get_transformation_matrix(0,0,np.pi/4,0,0,0.5)

print("Transformation matrix: \n",T)
# Create a homogenous coordinate vector for all points
X = np.vstack([x,y,z,np.ones(100)])

print("We now have 100 different homogenous coordinate vectors:",X.shape)


# Project into image plane
im_coords_scaled = K@T@X
im_coords = im_coords_scaled/im_coords_scaled[2,:]

plt.plot(im_coords[0,:],im_coords[1,:],'o')
plt.xlim(0,640)
plt.ylim(0,480)
plt.show()

im_1 = cv2.cvtColor(cv2.resize(cv2.imread('./test_images/Drone_1.jpg'),(400,400)), cv2.COLOR_BGR2GRAY)
im_2 = cv2.cvtColor(cv2.resize(cv2.imread('./test_images/Drone_2.jpg'),(400,400)), cv2.COLOR_BGR2GRAY)
# Initiate ORB detector
orb = cv2.ORB_create()

# Find the keypoints and descriptors
kp1, des1 = orb.detectAndCompute(im_1,None)
kp2, des2 = orb.detectAndCompute(im_2,None)

# Match keypoints
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1,des2)

# Sort matches
matches = sorted(matches, key = lambda x:x.distance)

# Draw first 20 matches.
plt.figure(figsize=(15,5))
img3 = cv2.drawMatches(im_1,kp1,im_2,kp2,matches[:],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
plt.imshow(img3)
plt.show()

# Find keypoint correspondences
X1 = np.vstack([kp1[match.queryIdx].pt for match in matches])
X2 = np.vstack([kp2[match.trainIdx].pt for match in matches])

# Estimate homograpahy using opencv -
#Hcv, mask = cv2.findHomography(X2, X1, cv2.RANSAC, 5.0)

# Create a matrix from 1 keypoint correspondence
def point_correspondance_matrix(x, y, u, v):
    row_1 = np.array([-x, -y, -1, 0, 0, 0, x * u, y * u, u])
    row_2 = np.array([0, 0, 0, -x, -y, -1, x * v, y * v, v])

    return (np.vstack([row_1, row_2]))


# Create a full matrix from multiple keypoint correspondences - technically this works with more than 4 points too
def four_point_correspondance_matrix(kp1, kp2):
    rows = []
    for i in range(kp1.shape[0]):
        rows.append(point_correspondance_matrix(kp1[i, 0], kp1[i, 1], kp2[i, 0], kp2[i, 1]))
    return np.vstack(rows)


# Find the homography using singular value decomposition
def find_homography(kp1, kp2):
    M = four_point_correspondance_matrix(kp1, kp2)

    U, S, Vt = np.linalg.svd(M)
    H = Vt[-1, :].reshape(3, 3)  # Null space corresponds to smallest singular value
    return H / H[2, 2]  # Typically normalise by the last element of the homography


# Find a homography using ransac
def ransac_homography(kp1, kp2, N=1000, max_error=5.0):
    best_consensus_size = 0
    best_H = np.eye(3)

    for j in range(N):

        # Pick 4 points at random
        bins = np.random.choice(kp1.shape[0], 4, replace=False)

        # Calculate Homography
        H = find_homography(kp1[bins, :], kp2[bins, :])

        # Project points using Homography
        X1_h = np.hstack((kp1, np.ones((kp1.shape[0], 1)))).T  # homogenous coordinates

        projected = H @ X1_h
        # Normalise (last column must be one for homogenous coordinates)
        X1_2 = projected[0:2, :] / projected[2, :]

        # Calculate reprojection error
        reprojection_errors = np.sqrt(np.sum(np.power(X2 - X1_2.T, 2), axis=1))

        # Count consensus set size
        consensus_size = np.sum(reprojection_errors < max_error)

        # Save current best homography
        if consensus_size > best_consensus_size:
            best_consensus_size = consensus_size
            best_H = H

    return best_H

H = ransac_homography(X1,X2,N=10000,max_error=5)
print(H)

# Let's apply the homography to our image - in theory it should generate the second picture
im_warp = cv2.warpPerspective(im_1,H,(im_1.shape[0],im_1.shape[1]))

plt.figure(figsize=(15,5))
plt.subplot(1,3,1)
plt.imshow(im_1)
plt.title('Image 1')
plt.subplot(1,3,2)
plt.imshow(im_warp)
plt.title('Image 1 warped by Homography')
plt.subplot(1,3,3)
plt.imshow(im_2)
plt.title('Image 2')
plt.show()