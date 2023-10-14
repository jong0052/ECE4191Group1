import numpy as np
import cv2
import matplotlib.pyplot as plt



#########################################################
#img = cv2.imread("original.jpg",cv2.IMREAD_GRAYSCALE)
#cap = cv2.VideoCapture(0)
#frame = cv2.imread("90 degree 200mm shift.jpg")
img = cv2.cvtColor(cv2.imread('original.jpg'),cv2.COLOR_BGR2GRAY)



th = 5*np.pi/100
tx=-50
ty=50
M = np.array([[np.cos(th),-np.sin(th),tx],[np.sin(th),np.cos(th),ty],[0,0,1]])
frame = cv2.warpPerspective(img,M,img.shape,borderMode=cv2.BORDER_REPLICATE)

sift = cv2.SIFT_create()
kp_image, desc_image = sift.detectAndCompute(img,None)

index_params = dict(algorithm =0, trees=5)
search_params = dict()

flann = cv2.FlannBasedMatcher(index_params, search_params)

#_,frame = cap.read()
#grayframe = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
grayframe = frame
kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe,None)
matches = flann.knnMatch(desc_image,desc_grayframe,k=2)

good_points = []

for m,n in matches:
    if(m.distance< 0.6*n.distance):
        good_points.append(m)

draw_params = dict(matchColor = (0,255,0)
                    ,singlePointColor = (255,0,0)
                    ,matchesMask = good_points,
                    flags = cv2.DrawMatchesFlags_DEFAULT)

query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_points]).reshape(-1,1,2)
train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1,1,2)
H, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC,5.0)
matches_mask = mask.ravel().tolist()

#Perspective transform
h,w = img.shape
pts = np.float32([[0,0],[0,h],[w,h],[w,0]]).reshape(-1,1,2)
dst = cv2.perspectiveTransform(pts,H)

homography = cv2.polylines(frame,[np.int32(dst)],True,(255,0,0),3)
cv2.imshow("Homography",homography)
cv2.waitKey(0)

print('The angle of rotation is %2.2f degrees.' % (np.arctan2(H[0, 1], H[0, 0]) * 180 / np.pi))
print('The translation is x=%2.2f, y=%2.2f pixels' % (H[0, 2], H[1, 2]))

################################################################################

# img1 = cv2.imread('original.jpg',cv2.IMREAD_GRAYSCALE)
# img2 = cv2.imread('200mm shift.jpg',cv2.IMREAD_GRAYSCALE)
#
# sift = cv2.SIFT_create()
#
# kp1, des1 = sift.detectAndCompute(img1,None)
# kp2, des2 = sift.detectAndCompute(img2,None)
#
# FLANN_INDEX_KDTREE = 1
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks=50)
#
# flann = cv2.FlannBasedMatcher(index_params,search_params)
#
# matches = flann.knnMatch(des1,des2,k=2)
#
# matchesMask = [[0,0] for i in range(len(matches))]
#
# for i,(m,n) in enumerate(matches):
#     if m.distance < 0.7*n.distance:
#         matchesMask[i] = [1,0]
#
# draw_params = dict(matchColor = (0,255,0)
#                    ,singlePointColor = (255,0,0)
#                    ,matchesMask = matchesMask,
#                    flags = cv2.DrawMatchesFlags_DEFAULT)
#
# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)
#
# plt.imshow(img3,), plt.show()
#



