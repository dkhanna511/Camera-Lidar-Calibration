import os
import shutil
import numpy as np
import random
import glob
import re 
import random
import argparse
# import defaults
import yaml
# freq_lidar = 10.0
# freq_camera = 7.0

with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)

def files_to_float(path):
    files_float = []
    for i in path:
        timestamp = i.split('.')[0:2]
        timestamp_float = float(['.'.join(timestamp)][0])
        files_float.append(timestamp_float)

    return files_float



def extract_image_per_sec(images_dir, camera_fps):
    print(len(os.listdir(images_dir)))

    count = 0
    images_list = []
    list = []
        
    for i in sorted(os.listdir(images_dir)):
        if count < camera_fps:
            count +=1
            # print("length of list : ", len(list))    
            list.append(i)
        elif count==camera_fps:
            count = 0
            # print("length of list : ", len(list))
            image_file = random.choice(list)
            list = []
            images_list.append(image_file)


    # print(images_list)
    
    # print(len(images_list))

    return images_list

def synchronize_files(image_path, pcd_path,  scaled_files_folder):

    path = extract_image_per_sec(image_path, config["camera_fps"])
    scaled_images_path = os.path.join(scaled_files_folder, "synced_images")
    if not os.path.exists(scaled_images_path):
        os.makedirs(scaled_images_path)
    for i in path:
        print(i)
        shutil.copy(os.path.join(image_path, i), os.path.join(scaled_images_path, i))


    floating_points = files_to_float(os.listdir(pcd_path))
    synced_files_path = os.path.join(scaled_files_folder, "synced_lidar_points")
    sync_file_ext = ".pcd"
    
    print("synced files path : ", synced_files_path)
    # exit(0)
    if not os.path.exists(synced_files_path):
        os.makedirs(synced_files_path)


    list = []
    print("path : ", path)
    for i, hello in enumerate(sorted(os.listdir(scaled_images_path))):
        # print(hello)
        timestamp = hello.split('.')[0:2]
        
        timestamp_float = float(['.'.join(timestamp)][0])
        
        closest_diff = 1000
        for num in floating_points:
            if abs(num - timestamp_float) < closest_diff:
                closest_diff = abs(num - timestamp_float)
                # count +=1
                closest = num
                # print("abs(num - timestamp_float) : {} and closest is : {}".format(abs(num - timestamp_float), closest))
                
        print("closest : {:.6f} ".format(closest))
        shutil.copy(os.path.join(pcd_path, str("{:.6f}".format(closest))+sync_file_ext), synced_files_path)# print("{:.6f}".format(closest))
    # print(closest)
        # break
    # break
    print("synced lidar points path : ", synced_files_path)
    print("length of images : ", len(os.listdir(path)))
    print("length of synced lidar points  : ", len(os.listdir(synced_files_path)))
    extract_file_per_sec(path, synced_files_path)
    # 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--images_dir', required=True, type=str, help='images_folder')
    parser.add_argument('--lidar_points_dir', required=True, type=str, help='lidar_points_folder')
    parser.add_argument('--output_dir', required=True, type=str, help='Output folder for synced files')
    #parser.add_argument('--camera_fps', required=True, type=float, help='Camera FPS')
    #parser.add_argument('--lidar_freq', required=True, type=float, help='Lidar Frequency')
    
    args = parser.parse_args()

    images_dir = args.images_dir
    lidar_points_dir = args.lidar_points_dir
    scaled_files_folder = args.output_dir

    synchronize_files(images_dir, lidar_points_dir, scaled_files_folder)
