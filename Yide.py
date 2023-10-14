import numpy as np
import cv2
import time






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


def find_homography(kp1, kp2):
    M = four_point_correspondance_matrix(kp1, kp2)

    U, S, Vt = np.linalg.svd(M)
    H = Vt[-1, :].reshape(3, 3)  # Null space corresponds to smallest singular value
    return H / H[2, 2]  # Typically normalise by the last element of the homography


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
        reprojection_errors = np.sqrt(np.sum(np.power(kp2 - X1_2.T, 2), axis=1))

        # Count consensus set size
        consensus_size = np.sum(reprojection_errors < max_error)

    # Save current best homography
        if consensus_size > best_consensus_size:
            best_consensus_size = consensus_size
            best_H = H

    return best_H