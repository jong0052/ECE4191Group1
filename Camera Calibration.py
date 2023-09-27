import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt


#set initial parameters for calibration
chessboardSize = (7,4)
frameSize = (1440,1080)
#camera calibration
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1],3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')

#run rach picture and find corners, and if existing, append the corner to both arrays
for image in images:
 #print(image)
 img = cv2.imread(image)
 gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
 ret, corners = cv2.findChessboardCorners(gray,chessboardSize,None)
 if ret == True:
   objpoints.append(objp)
   corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
   imgpoints.append(corners)
   cv2.drawChessboardCorners(img,chessboardSize,corners2,ret)
   #cv2.imshow('img',img)
   cv2.waitKey(1000)

cv2.destroyAllWindows()

#calibrate camera and display internal matrix and distorsion matrices
ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,imgpoints,frameSize,None,None)

print("Camera Calibrated: ",ret)
print("\nCamera Matrix:\n", cameraMatrix)
print("\nDistortion Parameters:\n", dist)
print("\nRotation Vectors:\n", rvecs)
print("\nTranslation Vectors:\n", tvecs)


#undistorsion
img = cv2.imread("image4.jpeg")
h,w = img.shape[:2]
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

dst = cv2.undistort(img,cameraMatrix,dist,None,newCameraMatrix)
#crop image
x,y,w,h = roi
dst = dst[y:y+h,x:x+w]
cv2.imwrite("undistorted.jpeg",dst)

#undistorsion with mapping
mapx,mapy = cv2.initUndistortRectifyMap(cameraMatrix,dist,None,newCameraMatrix,(w,h),5)
dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
#crop image
x,y,w,h = roi
dst = dst[y:y+h,x:x+w]
cv2.imwrite("undistored_mapped.jpeg",dst)
#reprojection error
mean_error=0

for i in range(len(objpoints)):
    imgpoints2,_ = cv2.projectPoints(objpoints[i],rvecs[i],tvecs[i],cameraMatrix,dist)
    error = cv2.norm(imgpoints[i],imgpoints2,cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print("\ntotal error: {}".format(mean_error/len(objpoints)))
print("\n\n\n")



