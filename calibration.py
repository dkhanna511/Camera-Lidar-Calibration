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
from scipy.spatial import distance

with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)


def find_lidar_centres(lidar_points):
    lidar_centres = []
    for i in range(len(lidar_points)):
        centre = (np.sum(lidar_points[i], axis = 0)/len(lidar_points[i])).reshape((3,1))
        lidar_centres.append(centre)

    return np.array(lidar_centres)


def rotation_calculation(A,B):
    
    
    num_rows, num_cols = A.shape;

    # # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    centroid_A = centroid_A.reshape((3,1))
    centroid_B = centroid_B.reshape((3,1))
    # A = A.reshape((3,1)) 
    # B = B.reshape((3,1)) 
    
    
    # # subtract mean
    Am = A - np.tile(centroid_A, (1, num_cols))
    Bm = B - np.tile(centroid_B, (1, num_cols))

    #dot is matrix multiplication for array
    #this term is for all the n planes that we have
    H = np.matmul(Am ,Bm.T)
    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T , U.T)
    if (np.linalg.det(R) < 0):
        S = [[1,0,0], [0,1, 0], [0, 0, np.linalg.det(R)]]
        R = np.dot(np.dot(Vt.T, S), U.T)
    
    return R


def load_data(path):
    
    path = path
    files=os.listdir(path)
    # print(files)
    data,images,lidar_points=[],[],[]
    image_path=os.path.join(path,files[0])
    lidar_points_path=os.path.join(path, files[1])
    images_folder=sorted(os.listdir(image_path))
    lidar_points_folder=sorted(os.listdir(lidar_points_path))
    
    # print("lidar points folder :" , lidar_points_folder)
    # print("images folder : ", images_folder)
    os.chdir(image_path)
    for i in images_folder:
        images.append(cv2.imread(i))
    
    # print(images)
    os.chdir(lidar_points_path)
    
    for j in lidar_points_folder:
        try:
            lidar_points.append(np.load(j))
        except:
            lidar_points.append(np.loadtxt(j))


    for d in data:
        images.append(d['data'][0][0][0])
        lidar_points.append(d['data'][0][0][1])
    
    return images,np.array(lidar_points)


def normal_in_camera_frame(rvecs):
    rotation = get_Rotation_Vectors(rvecs)
    normalInWorld = [0,0,1]
    normalInWorld = np.reshape(normalInWorld, (3,1))
    camera_normals = []
    distances = []
    
    for i in range(len(rotation)):
        
        vector = np.matmul(rotation[i],normalInWorld)
        camera_normals.append(vector)
        
    for i in range(len(camera_normals)):
        if camera_normals[i][2] > 0:
            # print("I have reached here as well")
            camera_normals[i] = -camera_normals[i]  
    
        
    return np.array(camera_normals)


def get_Rotation_Vectors(rvecs):

    rvecs = np.asarray(rvecs)
    rotation_vectors = []

    for i in range(len(rvecs)):
        rotation_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvecs[i], rotation_matrix)
        rotation_vectors.append(rotation_matrix)

    return rotation_vectors


def get_extrinsic_matrices(images, objpoints, imgpoints):

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # print("criteria " , criteria)
    w = 964
    h = 1288
    print("width and height of the image: ", w, h)

    
    for img in images:
        # print(img)
        
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

    
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners = np.reshape(corners, (48,2))
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)            
        # Draw and display the corners
        # cv2.drawChessboardCorners(img, (8,6), corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(500)
            #print(corners2)

    objpoints = [x * config["square_length_metres"] for x in objpoints]
    # print(objpoints[0])
    # cv2.destroyAllWindows()
    
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    # imgpoints = np.asarray(imgpoints)
    print("intrinsic parameters : {}".format(mtx))
    print("rvecs shape  : {}".format(np.shape(rvecs)))
    

    return np.array(rvecs), np.array(tvecs)
    # return np.array(rotations), np.array(translations)

def translation_calculation(sensor_to_centres, sensor_from_centres, sensor_to_normals, sensor_from_normals, rotation):
    
    n_nT = [0,0,0]

    for i in range(len(sensor_to_normals)):
        n_nT = n_nT + np.dot(sensor_to_normals[i], sensor_to_normals[i].T)
        
    
    A_ = n_nT
    
    val = [0,0,0]
    for i in range(len(sensor_to_normals)):
        n_nT = np.dot(sensor_to_normals[i], sensor_to_normals[i].T)
        centre = sensor_to_centres[i].T - np.dot(sensor_from_centres[i].T, rotation.T)
    
        val = val+np.dot(centre, n_nT)
    B_ = val

    # translation = np.dot(B_, np.linalg.inv(A_))
    # print("translation with taking inverse : ", translation)
    B_ = B_.reshape((3, 1))
    translation, _, _, _ = np.linalg.lstsq(A_, B_, rcond = None)
    # print("translation using least square formula : ",translation)
    return translation.T


def normal_in_lidar_frame(lidar_points):

    # Equation of a plane is ax + by +cz = d
    # Normals to the plane are given by a, b, and c values
    # Here we have x, y, z values as lidar points and we want to evaluate a, b, c
    # We use least square method where Ax = B is the equation of the plane
    # We take the value of c =-1
    # Here A  = [x, y, 1], B =  [z] So [x, y, 1][a, b, d] = [z]   ( xa + by -d = z)
    # After determining (a, b, d) we extract (a, b) and append "-1" in the array to obtain (a, b, -1) (Here we took c = -1)
    # we determine the unit normal vector by dividing (a, b, c) by sqrt(a^2 + b^2 + 1^2)
    normals = []
                                   
    for i in range(len(lidar_points)):
        point = lidar_points[i]
    
        variables = np.concatenate((point[:,0:2] , np.ones((np.shape(point)[0],1),dtype=int)), axis = 1)
        # print("rank of variables : ", np.linalg.matrix_rank(variables))
        rhs = point[:,2]    
        rhs = rhs.reshape(np.shape(rhs)[0],1)
        

        x = np.linalg.lstsq(variables, rhs, rcond=None)
        normal = np.concatenate((x[0][0:2], -1 * np.ones((1,1))))
        modVal = math.sqrt(math.pow(normal[0],2) + math.pow(normal[1],2) + math.pow(normal[2],2))
        normal = normal/modVal
        
        # If n.p0 where p0 is any point lying on the plane < 0 , this means n is in the direction of the plane, otherwise in the opposite direction.
        # We need this step since by fixing c = -1, we are fixing the direction of c. Hence we must check the direction of normal with respect to the plane.

        sign = np.matmul(normal.T,point[0].reshape((3,1)))
        if(sign<0):
            normal = 0-normal
            
        normals.append(normal)
    

    for i in range(len(normals)):
        if normals[i][0]> 0:
            normals[i] = -normals[i] 
        
        
    return np.array(normals)


def error_estimate(sensor_to_points,sensor_from_points, sensor_to_normals, sensor_from_normals, sensor_to_centres, sensor_from_centres, rotation, translation, string = "Camera to Lidar"):

    # estimating error for rotation matrix by determining the cosine distance between lidar_normals and transformed normals
    cosine_distance = []
    new_normals = []
    for i in range(len(sensor_from_normals)):
        new_normals.append(np.dot(rotation, sensor_from_normals[i]))
    new_normals = np.asarray(new_normals)
    for i in range(len(new_normals)):
        cosine_distance.append(distance.cosine(sensor_to_normals[i], new_normals[i]))

    #estimating translation offset by transforming all camera points in lidar frame and determing the root mean squared error.
    ave_val_lhs = []
    for i in range(len(sensor_to_points)):
        points = sensor_to_points[i]
        val = np.dot(sensor_to_normals[i].T, points.T)
        ave_val_lhs.append(np.sum(val)/len(points))
    
    
    ave_val_rhs = []
    
    for i in range(len(sensor_from_points)):
        rotation_vec = np.dot(rotation, sensor_from_points[i].T)
        translation_vec = np.ones((1,len(sensor_from_points[i]))) * translation.T
        
        inner_add = rotation_vec  + translation_vec
        total_prod = np.dot(sensor_to_normals[i].T, inner_add)
        ave_val_rhs.append(np.sum(total_prod)/len(sensor_from_points[i]))
        
    final_sub = []
    for i in range(len(sensor_from_points)):
        final_sub.append(np.square(ave_val_lhs[i] - ave_val_rhs[i]))

     
    # print("mean squared error {} : {} ".format(string, np.mean(final_sub)))

    return np.mean(cosine_distance), np.sqrt(np.mean(final_sub))


def normalize_points(lidar_points):
    lidar_centres = []
    centred_points = []
    average_norms = []
   
    for i in range(len(lidar_points)):
        points = lidar_points[i]

        centres = np.mean(points, axis = 0)
        
        new_centres  =np.ones((len(lidar_points[i]),1)) * centres
        
        centred_points.append((points - new_centres))
    
    for i in range(len(centred_points)):
        norm_axis = []
        for j in range(len(centred_points[i])):
            norm_axis.append(np.linalg.norm(centred_points[i][j]))
              
        average_norms.append(np.mean(norm_axis))
    

    new_points = []
    for i in range(len(centred_points)):
        new_points.append(centred_points[i]/ average_norms[i])
    
    return np.array(new_points)    

def plot_graphs(sensor_to_normals, sensor_from_normals, rotation):
    index=0
    for sensor_to_normal,sensor_from_normal in zip(sensor_to_normals,sensor_from_normals):
        fig=plt.figure()
        ax=fig.gca(projection='3d')
        rotatedVector=np.dot(rotation,sensor_from_normal)
        #x, y, z = np.meshgrid(np.arange(-0.8, 1, 0.2),np.arange(-0.8, 1, 0.2),np.arange(-0.8, 1, 0.8))
        ax.quiver(0,0,0,sensor_from_normal[0],sensor_from_normal[1],sensor_from_normal[2],length=0.1,normalize=True,color="blue")
        ax.quiver(0,0,0,sensor_to_normal[0],sensor_to_normal[1],sensor_to_normal[2],length=0.1,normalize=True,color="red")
        #ax.quiver(pts[0][0],pts[0][1],pts[0][2],normal_camera_approx[0],normal_camera_approx[1],normal_camera_approx[2],length=0.1,normalize=True,color="green")
        ax.quiver(0,0,0,rotatedVector[0],rotatedVector[1],rotatedVector[2],length=0.1,normalize=True,color="green")
        #fig= plt.figure(figsize = (6,6))
        #fig.add_subplot(3, i, 1)
        plt.show()
        if(index == 10):
            break
        index+=1


def transform_world_to_camera(objpoints, rvecs, tvecs):
    rotation_vectors = get_Rotation_Vectors(rvecs)
    transformed_points = []
    # print("shape of objpoints : ", np.shape(objpoints))
    # print("shape of tvecs : ", np.shape(tvecs))
    objpoints = [x * config["square_length_metres"] for x in objpoints]
    # print(objpoints[0])
    

    for i in range(len(objpoints)):
        points = objpoints[i].T
        translation = np.ones((1,len(objpoints[i]))) * tvecs[i]
        transformation = np.dot(rotation_vectors[i], points) + translation
        transformed_points.append(transformation.T)
        
    transformed_points = np.asarray(transformed_points)
    return transformed_points

def transformation(sensor_to_points, sensor_from_points, rotation, translation, plot_transformation):
    
    transformed_points = []
    
    for i in range(len(sensor_from_points)):
        points = sensor_from_points[i]
        # print("shape of points : ", np.shape(camera_points[i]))
        translation_vec = np.ones((1,len(sensor_from_points[i]))) * translation_CTL.T
        # print("shape of translation : ", np.shape(translation))
        transformation = np.dot(rotation, points.T) + translation_vec
        # transformation  = np.dot(rotation_vectors[i].T, (points - translation))
        transformed_points.append(transformation.T)
    # print(transformed_points)
    if plot_transformation ==True:
        for i in range(len(sensor_from_points)):
                ax = plt.axes(projection='3d')    
                transformed_points = np.asarray(transformed_points)

                ax.scatter(transformed_points[i][:,0], transformed_points[i][:,1], transformed_points[i][:,2], c = "black", s=5)  
                ax.scatter(sensor_to_points[i][:,0], sensor_to_points[i][:,1], sensor_to_points[i][:,2], c = "red", s=1)        
                plt.savefig(os.path.join(args.output_dir, "transformations"))
                plt.show()
    
    return transformed_points

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--training_dir', required=False, type=str, help='training_folder')
    parser.add_argument('--testing_dir', required=False, type=str, help='testing_folder')
    parser.add_argument('--output_dir', required = True, type=str, help='Folder to save rotation and translation vectors')
    
    args = parser.parse_args()

    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    if(args.training_dir):
        images,lidar_points_orig= load_data(args.training_dir)

        # lidar_points_orig = [x * 100 for x in lidar_points_orig]

        
    elif(args.testing_dir):
        images,lidar_points_orig= load_data(args.testing_dir)
    lidar_points= normalize_points(lidar_points_orig)
    # print(lidar_points)
    lidar_points_orig = np.asarray(lidar_points_orig)
    # print(lidar_points_orig)
    rvecs, tvecs = get_extrinsic_matrices(images, objpoints, imgpoints)
    # print(rvecs, tvecs)
    camera_points = transform_world_to_camera(objpoints, rvecs, tvecs)
    
    
    camera_normals= normal_in_camera_frame(rvecs)

    lidar_normals = normal_in_lidar_frame(lidar_points)
    lidar_normals_orig = normal_in_lidar_frame(lidar_points_orig)

    camera_centres =  tvecs#find_lidar_centres(camera_points)
    lidar_centres = find_lidar_centres(lidar_points_orig)
    
    A=np.asarray(camera_normals[0])
    for i in range(1,len(camera_normals)):
        A=np.hstack((A,camera_normals[i]))


    B=np.asarray(lidar_normals[0])
    for i in range(1,len(lidar_normals)):
        B=np.hstack((B,lidar_normals[i]))


    if(args.training_dir):
        rotation_CTL = rotation_calculation(A, B)
        np.savetxt(os.path.join(args.output_dir, "rotation_CTL.npy"),rotation_CTL)
        rotation_LTC = rotation_calculation(B, A)
        np.savetxt(os.path.join(args.output_dir, "rotation_LTC.npy"),rotation_LTC)
        
        translation_CTL = translation_calculation(lidar_centres, camera_centres, lidar_normals, camera_normals, rotation_CTL)
        np.savetxt(os.path.join(args.output_dir, "translation_CTL.npy"),translation_CTL)
        # exit(1)
        translation_LTC = translation_calculation(camera_centres, lidar_centres, camera_normals,  lidar_normals_orig, rotation_LTC)
        np.savetxt(os.path.join(args.output_dir, "translation_LTC.npy"),translation_LTC)

        
    elif(args.testing_dir):
        rotation_CTL = np.loadtxt(os.path.join(args.output_dir, "rotation_CTL.npy"))
        translation_CTL = np.loadtxt(os.path.join(args.output_dir,"translation_CTL.npy"))
        translation_CTL = np.reshape(translation_CTL, (1, 3))
        rotation_LTC = np.loadtxt(os.path.join(args.output_dir, "rotation_LTC.npy"))
        translation_LTC = np.loadtxt(os.path.join(args.output_dir,"translation_LTC.npy"))
        translation_LTC = np.reshape(translation_LTC, (1, 3))
        

    cam_points = transformation(lidar_points_orig, camera_points, rotation_CTL, translation_CTL, plot_transformation = config["plot_cam_lidar_plots"])
    
    print("rotation CTL : ", rotation_CTL)
    
    print("translation CTL : ", translation_CTL)
    
    print("rotation LTC : ", rotation_LTC)
    
    print("translation LTC : ", translation_LTC)
    
    cosine_distance, offset_distance = error_estimate(lidar_points_orig,camera_points , lidar_normals, camera_normals, lidar_centres, camera_centres, rotation_CTL, translation_CTL )

    print("cosine distane normal CTL: ", cosine_distance)
    print("translation offset CTL : ", offset_distance)

    cosine_distance, offset_distance = error_estimate(camera_points, lidar_points_orig,  camera_normals, lidar_normals, camera_centres, lidar_centres, rotation_LTC, translation_LTC, string = "Lidar To Camera")


    print("cosine distane normal LTC: ", cosine_distance)
    print("translation offset LTC : ", offset_distance)

    if (config["plot_cam_lidar_normals"]):
        plot_graphs(camera_normals, lidar_normals, rotation_LTC)
