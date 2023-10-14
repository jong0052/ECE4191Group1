import numpy as np
import cv2
from matplotlib import pyplot as plt
from time import sleep
import time
from PIL import Image, ImageFilter

# Load our image
# im = cv2.cvtColor(cv2.imread('Drone_1.jpg'), cv2.COLOR_BGR2GRAY)
# # Rotate 27 degrees, tranlate -15,5 pixels
# th = 27*np.pi/180
# tx=-15
# ty =5
# M = np.array([[np.cos(th),-np.sin(th),tx],
#              [np.sin(th),np.cos(th),ty],
#               [0,0,1]])
# # Simulate the change in view that would we would undero given this transformation
# im_new = cv2.warpPerspective(im,M,im.shape,borderMode=cv2.BORDER_REPLICATE)

#Webcam matrix values
cameraMatrix = np.array([[1.66911791e+03,0.00000000e+00,6.66625922e+02],[0.00000000e+00,1.70586125e+03,5.63514139e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]])
#dist = [[ 0.17923726,-1.39938755,0.06018298,-0.03523539,2.38244121]]

#Picamera matrix values
#cameraMatrix = [[2.40140558e+03,0.00000000e+00,1.04908260e+03],[0.00000000e+00,2.37984043e+03,9.84406691e+02],[0.00000000e+00,0.00000000e+00,1.00000000e+00]]
#dist = [[ 0.03913461,0.35405608,0.00330635,-0.03732458,-0.9110321 ]]



#simulate = False


def homography(im, im_new):
    # Initiate ORB detector
    orb = cv2.ORB_create(nfeatures=150,scaleFactor=2,nlevels=8,edgeThreshold=31,firstLevel=0,WTA_K=2,patchSize=70,fastThreshold=10)
    #orb = cv2.SIFT_create()
    # Find the keypoints and descriptors
    kp1, des1 = orb.detectAndCompute(im, None)
    kp2, des2 = orb.detectAndCompute(im_new, None)

    # SIFT
    # FLANN_INDEX_KDTREE = 1
    # index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    # search_params = dict(checks=50)
    #
    # flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(des1, des2, k=2)
    # good = []
    # for m, n in matches:
    #     if m.distance < 0.7 * n.distance:
    #         good.append(m)
    #
    # src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    # dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    # H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    # matchesMask = mask.ravel().tolist()
    # h, w = im.shape
    # pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
    # dst = cv2.perspectiveTransform(pts, H)
    # im = cv2.polylines(im, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)


    # Match keypoints
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    # Sort matches
    matches = sorted(matches, key=lambda x: x.distance)

    # Draw first 20 matches.
    plt.figure(figsize=(15, 5))
    img3 = cv2.drawMatches(im, kp1, im_new, kp2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    plt.imshow(img3)
    plt.show()

    # Find keypoint correspondences
    X1 = np.vstack([kp1[match.queryIdx].pt for match in matches])
    X2 = np.vstack([kp2[match.trainIdx].pt for match in matches])

    # Estimate homograpahy using opencv -
    H, mask = cv2.findHomography(X1, X2, cv2.RANSAC, 1.0)

    # Lets apply the homography to our image - in theory it should generate the second picture
    im_warp = cv2.warpPerspective(im, H, (im.shape[0], im.shape[1]))
    num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, cameraMatrix)
    #print(num)
    #print('H', H)
    #print('Map', M)
    #print(len(Rs[0][1]))
    #print(Rs)
    #print(Ts)
    Rsx=0
    Rsy = 0
    count=0
    for i in range(len(Rs)):
        Rsx += Rs[i][0][1]
        Rsy += Rs[i][0][0]

    #print('The angle of rotation is %2.2f degrees.' % (np.arctan2(Rsx/num, Rsy/num) * 180 / np.pi))
    #print('The angle of rotation is %2.2f degrees.' % (np.arctan2(Rs[2][0][1], Rs[2][0][0]) * 180 / np.pi))
    print('The angle of rotation is %2.2f degrees.' % (np.arctan2(H[0, 1], H[0, 0]) * 180 / np.pi))
    print('The translation is x=%2.2f, y=%2.2f pixels' % (H[0, 2], H[1, 2]))
    #print('The translation is x=%2.2f, y=%2.2f pixels' % (Ts[2][0], Ts[2][1]))
    #print(num)
    #print(Rs)
    #print(Ts)

# cam = cv2.VideoCapture(0)
# _,im_old = cam.read()
# kernel = np.ones((5,5),np.uint8)
# im_old = cv2.Canny(im_old,50,150)
# im_old = cv2.morphologyEx(im_old, cv2.MORPH_OPEN, kernel)
# #im_old = cv2.cvtColor(im_old,cv2.COLOR_BGR2GRAY)
# count = 0
#while True:

    # if simulate == True:
    #
    #     th = 27*np.pi/180
    #     tx=-15
    #     ty =5
    #     M = np.array([[np.cos(th),-np.sin(th),tx],
    #                  [np.sin(th),np.cos(th),ty],
    #                   [0,0,1]])
    #     # Simulate the change in view that would we would undero given this transformation
    #     im_new = cv2.warpPerspective(im, M, im.shape, borderMode=cv2.BORDER_REPLICATE)
    #else:


    #cam = cv2.VideoCapture(0,cv2.CAP_DSHOW)

    #################################
    # time.sleep(2)
    # _,im = cam.read()
    # im = cv2.Canny(im,50,150)
    # kernel = np.ones((5,5),np.uint8)
    # im = cv2.morphologyEx(im, cv2.MORPH_OPEN, kernel)
    # #im = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    #
    # #change the amount of time between shots
    #
    # start_time = time.time()

    #homography(im_old,im)
    #im_old = im
    ################################



    #print(count)
    #plt.subplot(1,2,1)
    #plt.imshow(im_old)
    #plt.subplot(1,2,2)
    #plt.imshow(im)
    #plt.show()


im = cv2.imread('original.jpg')
im = cv2.blur(im,(2,2))
im = cv2.fastNlMeansDenoisingColored(im,None,20,10,7,21)
im = cv2.Canny(im,0,100)
#im = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
th = -0*np.pi/100
tx=-0
ty=0
M = np.array([[np.cos(th),-np.sin(th),tx],[np.sin(th),np.cos(th),ty],[0,0,1]])

#im_new = cv2.imread('originalRotated.jpg')
#im_new = cv2.warpPerspective(im,M,im.shape,borderMode=cv2.BORDER_REPLICATE)
im_new = cv2.imread('originalRotated.jpg')
im_new = cv2.blur(im_new,(2,2))
im_new = cv2.fastNlMeansDenoisingColored(im_new,None,20,10,7,21)
im_new = cv2.Canny(im_new,0,100)
homography(im,im_new)
