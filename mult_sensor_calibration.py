import numpy as np
import cv2
import glob
import os
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import scipy.spatial.distance
import random
import argparse
# import yaml
from sklearn.metrics import mean_squared_error
# import defaults
import yaml
import time
from scipy.spatial import distance

def get_extrinsic_matrices(images, objpoints, imgpoints):

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # print("criteria " , criteria)
    w = 964
    h = 1288
    print("width and height of the image: ", w, h)


    rotations = []
    translations = []
    
    for file in os.listdir(path):
        
        start = time.process_time()
        img = cv2.imread(os.path.join(path, file))
        print(file)
        # print(img)

        
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # cv2.imshow("frame", gray)
        # cv2.waitKey(0)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)
        print(ret)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners = np.reshape(corners, (48,2))
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
            # ret_,rvecs, tvecs = cv2.solvePnP(objpoints, corners2, mtx, dist)
            # # print(rvecs, "\n")
        # Draw and display the corners
            # cv2.drawChessboardCorners(img, (8,6), corners2, patternWasFound = True)
            # cv2.imshow('img', img)
            # cv2.waitKey(0)
            #print(corners2)
        # else:
        #     cv2.waitKey(500)
        #     continue
            end = time.process_time()
            print("time taken : {} and checkerboard found".format(end - start))
        end = time.process_time()
        print("time taken : {}".format(end - start))
 
        # cv2.destroyAllWindows()


    return None, None
    # return np.array(rotations), np.array(translations)

objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


path = "/media/dheeraj/New Volume/IIITD_ALIVE/extracted_images"

get_extrinsic_matrices(path, objpoints, imgpoints)