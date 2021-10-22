import numpy as np 
import open3d as o3d
from numpy import exp, abs, angle
import math
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import axes3d, Axes3D
import os
import copy
# import defaults
from open3d import *
import argparse
from scipy.spatial import distance
import shutil
import yaml

with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)

#Convert cartesian coordinates to polar coordinates
def cart2pol(out_arr):
    arr = []
    for i in range(len(out_arr)):

        rho = (np.sqrt(out_arr[i][0]**2 + out_arr[i][1]**2))
        phi = 180* (np.arctan(out_arr[i][1]/out_arr[i][0]))/math.pi
        arr.append([rho, phi, out_arr[i][2]])
        # print(arr)

    return np.array(arr)
#Convert Polar Coordinates to Cartesian Coordinates
def pol2cart(arr):
    output = []
    for i in range(len(arr)):

        x = arr[i][0] * np.cos(arr[i][1]* math.pi/180) 
        y = arr[i][0] * np.sin(arr[i][1]* math.pi/180)
        output.append([x,y ,arr[i][2] ])

    return np.array(output)


def filter(arr):
    final = []
    for i in range(len(arr)):

        if arr[i][1] > config["azimuth_angle_degrees"]["lower_val"] and arr[i][1] < config["azimuth_angle_degrees"]["upper_val"] and arr[i][0] > config["depth_threshold_metres"]["lower_val"] and arr[i][0]<config["depth_threshold_metres"]["upper_val"]:
            final.append(arr[i])
            
            
            
    final = np.asarray(final)
    return final


def find_lidar_centres(lidar_points, board_width, board_height, inliers):
    lidar_points = lidar_points[inliers]
    lidar_centres = []
    print("length of lidar points is : {}".format(len(lidar_points)))
    centre = (np.sum(lidar_points, axis = 0)/len(lidar_points))
    # lidar_centres.append(centre)
    # print(centre[0])
    f_lidar_points = []
    # print(lidar_points[0][0])
    # print("length of lidar points : ", len(lidar_points))

    for i in range(len(lidar_points)):
        # print("lidar_points : ", lidar_points[i][0])
        if (lidar_points[i][1] < (centre[1] - board_width/2)) or (lidar_points[i][1] > (centre[1] + board_width/2)):
            # lidar_points = np.delete(lidar_points, [i], axis  = 0)
            continue
        if (lidar_points[i][2] < (centre[2] - board_height/2)) or (lidar_points[i][2] > (centre[2] + board_height/2)):
            # lidar_points = np.delete(lidar_points, [i], axis  = 0)
            continue

        f_lidar_points.append(lidar_points[i])
        
    
    del_inliers = []
    for i, j in enumerate(inliers):
        if j == True:
            del_inliers.append(i)
    #lidar_points = lidar_points_copy
    print("del_inliers count : {}".format(len(del_inliers)))

    # for i in del_inliers:
    #     # np.delete(lidar_points, )

    inliers = np.delete(inliers, del_inliers)
    
    print("Length of inliers : {}".format(len(inliers)))
    # print("length of lidar points : ", len(f_lidar_points))
    return np.array(f_lidar_points), inliers

def get_cosine_distance(lidar_points):

    normal_to_lidar = [0, 0, 1]
    cosine_distance = distance.cosine(lidar_points,normal_to_lidar )
    print("cosine_distance : {}".format(cosine_distance))

# global count
# count= 0

#Function to calculate the cosine distance between the Lidar plane detected and the Lidar placed normal.
def get_normals(lidar_points, pcd_file, save_plots):
    normals = []
    #defininig normal to the ground
    lidar_normal = [1, 0, 0]                             
    point = lidar_points

    #print("cosine_distance : {}".format(cosine_distance))
    try:
        variables = np.concatenate((point[:,0:2] , np.ones((np.shape(point)[0],1),dtype=int)), axis = 1)
    except:
        return 0
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
        
    # if normal[0]> 0:
    #     normal = -normal 
    
    cosine_distance = distance.cosine(normal,lidar_normal )
    
    return cosine_distance

def apply_ransac(lidar_points, pcd_file, save_plots):
    #residual threshold is the median absolute deviation of the target values 
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(lidar_points)
    plane_model, inliers = pcd.segment_plane(distance_threshold=config["residual_thresh_ransac_metres"], ransac_n=config["min_points_ransac"], num_iterations=config["num_iter_ransac"])
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    inlier_cloud.paint_uniform_color([0, 0, 1])
    outlier_cloud.paint_uniform_color([1, 0, 0])
    # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    print("##################Ransac function ################# ")
    print("shape of inliers is : {}".format(np.shape(inlier_cloud.points)))
    print("shape of outliers is : {}".format(np.shape(outlier_cloud.points)))
    
    print("shape of lidar points : {}".format(np.shape(lidar_points)))
    print("###################################################")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(np.array(inlier_cloud.points)[:,0], np.array(inlier_cloud.points)[:,1], np.array(inlier_cloud.points)[:,2], c='b',
                marker='o', label='Inlier data', s = 1)
    ax.scatter(np.array(outlier_cloud.points)[:,0], np.array(outlier_cloud.points)[:,1], np.array(outlier_cloud.points)[:,2], c='r',
                marker='o', label='Outlier data', s = 1)
    
    ax.legend(loc='lower left')
    plt.xlabel('pcd file : {}'.format(pcd_file))
    plt.savefig(os.path.join(save_plots, str(pcd_file.split(".pcd")[0])+"_plot"+".png"))
    # plt.show()

    return inliers
def filter_inliers(lidar_points, inliers):
    print("##############Filtering inliers function################# ")
    del_inliers = []
    lidar_points_copy = lidar_points.copy()
    print("len of lidar points = {}".format(len(lidar_points)))
    # print("len of inliers = {}".format(len(inliers)))
    # print("del_inliers_count = {}".format(del_inliers_count))
    
    for i, j in enumerate(inliers):
        if j == True:
            del_inliers.append(i)
    #lidar_points = lidar_points_copy
    print("del_inliers count : {}".format(len(del_inliers)))

    # for i in del_inliers:
    #     # np.delete(lidar_points, )

    lidar_points = np.delete(lidar_points, del_inliers, axis = 0)

    print("after length of lidar points : {}".format(len(lidar_points)))
    print("##########################################################")
    return lidar_points

def filter_pcd(path_pcd, images_dir_left, images_dir_right, save_dir_lidar, save_dir_images_left,save_dir_images_right , save_plots):

    # cosine_distance = 2.0
    board_width = 0.108*6#+0.098 + 0.122
    board_height = 0.108*8#+0.141+0.202


    if not os.path.exists(save_dir_lidar):
        os.makedirs(save_dir_lidar)

    if not os.path.exists(save_dir_images_left):
        os.makedirs(save_dir_images_left)
    


    if not os.path.exists(save_dir_images_right):
        os.makedirs(save_dir_images_right)
    if not os.path.exists(save_plots):
        os.makedirs(save_plots)

    # f_normals = []
    counter_cosine_great = 0
    for i, (pcd_file, image_file_left, image_file_right) in enumerate(zip(sorted(os.listdir(path_pcd)), sorted(os.listdir(images_dir_left)), sorted(os.listdir(images_dir_right)))):
        # print()
        print(pcd_file, "\n\n\n\n\n")
        file= os.path.join(path_pcd, pcd_file)

        pcd = o3d.io.read_point_cloud(file)
        np_arr = np.asarray(pcd.points)  
        # print(np_arr)

        cart_to_cylin = cart2pol(np_arr)

        filtered_arr = filter(cart_to_cylin)
        cylin_to_cart = pol2cart(filtered_arr)
        print(cylin_to_cart)
        # robustly fit line only using inlier data with RANSAC algorithm
        inliers = apply_ransac(cylin_to_cart, pcd_file, save_plots)
        # print("cylin_to_cart : {}".format(cylin_to_cart[0:10]))
        # print("inliers : {}".format(inliers[0:10]))
        counter = 0
        for i in inliers:
            if i==True:
                counter +=1
        #exit(0)
        print("counter inliers is : {}".format(counter))
        fin_lidar_points, inliers_ = find_lidar_centres(cylin_to_cart, board_width, board_height, inliers)
          
        
        # print("counter val is : {}".format(counter))
        #exit(0)
        # print(lidar_centres)
        # fin_lidar_points = find_lidar_centres(cylin_to_cart[inliers], board_width, board_height)
        cosine_distance = get_normals(fin_lidar_points, pcd_file, save_plots)
        print("cosine distance : {}".format(cosine_distance))
        # exit(0)
        print("length of inliers here is : {}".format(len(inliers_)))
        # print("Length of array here is : {}".format(len(cylin_to_cart)))
        fin_lidar_points_s = cylin_to_cart.copy()
        # fin_lidar_points_l = cylin_to_cart[inliers].copy()
        #f_normals.append(f_lidar_points)

        
        if cosine_distance > 0.7:
            counter_cosine_great +=1

        count_iteration_while = 0
        while cosine_distance > 0.7:
            count_iteration_while +=1
            print("cosine distance not right \n")
            fin_lidar_points = filter_inliers(fin_lidar_points, inliers_)
            inliers = apply_ransac(fin_lidar_points, pcd_file, save_plots)
            fin_lidar_points, inliers_ = find_lidar_centres(fin_lidar_points, board_width, board_height, inliers)
            cosine_distance = get_normals(fin_lidar_points, pcd_file, save_plots)
            # fin_lidar_points_s = fin_lidar_points_l
            print("cosine distance : {}".format(cosine_distance))
            if count_iteration_while ==5:
                break
        if count_iteration_while ==5:
            continue
        
        # fin_lidar_points = find_lidar_centres(fin_lidar_points_l, board_width, board_height)
            
        #get_cosine_distance(f_normals)

        print("here")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(fin_lidar_points)
        o3d.io.write_point_cloud(os.path.join(save_dir_lidar, pcd_file), pcd)
        shutil.copy(os.path.join(images_dir_left, image_file_left),os.path.join(save_dir_images_left, image_file_left))
        shutil.copy(os.path.join(images_dir_right, image_file_right),os.path.join(save_dir_images_right, image_file_right))
    # break
    
    print("The number of files for which Cosine distance was greater is : {}".format(counter_cosine_great))
# rosrun image_view image_saver image:=/camera/image_color


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--lidar_points_dir', required=True, type=str, help='lidar_points_folder')
    parser.add_argument('--images_dir', required=True, type=str, help='images_folder')
    
    parser.add_argument('--output_dir_lidar', required=True, type=str)
    parser.add_argument('--output_dir_images', required=True, type=str)
    
    parser.add_argument('--plots_dir', required=True, type=str, help='folder with plots after filtering')
    
    args = parser.parse_args()
    images_dir = args.images_dir
    path_pcd = args.lidar_points_dir
    save_dir_lidar = args.output_dir_lidar
    save_plots = args.plots_dir
    save_dir_images =  args.output_dir_images
    
    filter_pcd(path_pcd, images_dir, save_dir_lidar, save_dir_images, save_plots)



#background subtraction
#place the car, take 2-3 scans, each point in the background scan, take 5cm radius around each point, and compare it with the new scan. 

#get an estimate of the height of the lidar. and the angle from ground plane. estimate the exact orientation and equation of the plane.

#get the equation of the ground plane from lidar coordinate system
# ground plane - some normal and some offset
# easily compute that, normal and and offset
# use that for the ipm