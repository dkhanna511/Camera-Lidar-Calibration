import os
import numpy as np
import subprocess
# from pathlib import Path
import shutil
import argparse
from open3d import *
import glob
import random
# import defaults
import yaml

with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)


def extract_bag_file(output_dir, bag_file):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    bag_file_path = os.path.join(output_dir, bag_file)
    call_string = "rosbag record {} {} --duration={} -O {}".format(config["ros_topics"]["camera_topic"], config["ros_topics"]["lidar_topic"], config["duration_sec"], bag_file_path)
    print(call_string)
    subprocess.call(call_string, shell = True)
    return bag_file_path
def extract_sensor_messages(output_dir, bag_file):
    # print(images_dir, lidar_points_dir, bag_file)
    # exit(0)
    images_dir = os.path.join(output_dir, "images")
    lidar_points_dir = os.path.join(output_dir, "lidar_points")
    # print(images_dir, lidar_points_dir, bag_file)
    
    call_string = "python3 extract_sensor_msgs.py --images_dir {} --lidar_points_dir {} --bag_file {}".format(images_dir,lidar_points_dir,bag_file)
    
    print(call_string)
   
    # exit(0)
    subprocess.call(call_string, shell = True)
    call_string = "rosrun pcl_ros bag_to_pcd {} {} {}".format(bag_file, config["ros_topics"]["lidar_topic"], lidar_points_dir)
    subprocess.call(call_string, shell = True)
    
    return images_dir, lidar_points_dir
    
def timestamps(images_dir, lidar_points_dir, output_dir):
    call_string = "python3 timestamps.py --images_dir {} --lidar_points_dir {} --output_dir {}".format(images_dir,lidar_points_dir, output_dir)
    print(call_string)
    subprocess.call(call_string, shell = True)
    # print("camera fps : ", camera_fps)
    # print("lidar frequence : ", lidar_freq)
    # if camera_fps > lidar_freq:
    #     print("hello world 2")
    #     synced_images_dir = os.path.join(output_dir, "synced_images")
    #     synced_lidar_points_dir = os.path.join(output_dir, "lidar_points")
    # elif camera_fps <  lidar_freq:
    #     print("hello world")
    #     synced_images_dir = os.path.join(output_dir, "images")
    #     synced_lidar_points_dir = os.path.join(output_dir, "synced_lidar_points")
    synced_images_dir = os.path.join(output_dir, "synced_images")
    synced_lidar_points_dir = os.path.join(output_dir, "synced_lidar_points")

    return synced_lidar_points_dir, synced_images_dir
    

def image_select_gui(images_dir, lidar_points_dir, output_dir):

    call_string = "python3 image_select_gui.py --images_dir {} --lidar_points_dir {} --output_dir {}".format(images_dir,lidar_points_dir, output_dir)
    print(call_string)
    subprocess.call(call_string, shell = True)
    
    lidar_points_dir = os.path.join(output_dir, "extracted_lidar_points")
    images_dir = os.path.join(output_dir, "extracted_images")

    return lidar_points_dir, images_dir



def filter_pcd_files(lidar_points_dir, images_dir, filtered_lidar_points_dir, filtered_images_dir, plots_dir):

    call_string = "python3 filter_pcd.py --lidar_points_dir {} --images_dir {} --output_dir_lidar {} --output_dir_images {} --plots_dir {}".format(lidar_points_dir,images_dir, filtered_lidar_points_dir,filtered_images_dir ,plots_dir)
    print(call_string)
    subprocess.call(call_string, shell = True)


def renamer(file_dir , file_ext):
    print(sorted(os.listdir(file_dir)))
    files = sorted(glob.glob(os.path.join(file_dir, "*."+file_ext)))
    # print("files : ", files)

    # print(len(files))
    print(type(file_ext))
    print(file_ext)
    name_prefix = "FILE"
    for i, f in enumerate(files):
        # print(i)
        # print(f)
        new_name = "{}_{}.{}".format(name_prefix, str(i).zfill(2), file_ext)
        new_f = os.path.join(os.path.split(f)[0], new_name)
        print(new_name)
        os.rename(f, new_f)


def random_sampling(images_dir, lidar_points_dir, training_dir, validation_dir):
    filenames = ["images", "lidar_points"]

    for file in filenames:
        if not os.path.exists(os.path.join(training_dir,file)):
            os.makedirs(os.path.join(training_dir, file))
            os.makedirs(os.path.join(validation_dir, file))
    
    
    num_train = random.sample(range(len(os.listdir(images_dir))), config["num_samples_train"])
    print(len(num_train))
    print(len(images_dir))
    print(num_train)
    # print(os.listdir(images_dir))
    for index, (image, lidar_point) in enumerate(zip(sorted(os.listdir(images_dir)), sorted(os.listdir(lidar_points_dir)))):
        file_image = os.path.join(images_dir, image)
        file_lidar = os.path.join(lidar_points_dir, lidar_point)
        if index in num_train:
            shutil.copy(file_image, os.path.join(training_dir, filenames[0], image))
            shutil.copy(file_lidar, os.path.join(training_dir,filenames[1], lidar_point))

        else:
            shutil.copy(file_image, os.path.join(validation_dir,filenames[0], image))
            shutil.copy(file_lidar, os.path.join(validation_dir,filenames[1], lidar_point))



    # print(sorted(os.listdir(os.path.join(training_dir, "images"))), "\n\n")

    # print(sorted(os.listdir(os.path.join(validation_dir, "images"))), "\n\n")

def calibration(training_dir, testing_dir, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    print("##########Calibration results for training##########\n")
    call_string_train = "python3 calibration.py --training_dir {} --output_dir {}".format(training_dir, output_dir)
    
    subprocess.call(call_string_train, shell = True)
    print("##########Calibration results for testing##########\n")
    call_string_test = "python3 calibration.py --testing_dir {} --output_dir {}".format(testing_dir, output_dir)
    subprocess.call(call_string_test, shell = True)
def pcd_to_numpy(path_pcd):
    # print(sorted(os.listdir(path_pcd)))
    # listdir = glob.glob(os.path.join(path_pcd, "*.pcd"))

    # print(listdir)
    for i, pcd_file in enumerate(sorted(os.listdir(path_pcd))):
        # print()
        file= os.path.join(path_pcd, pcd_file)

        pcd = open3d.io.read_point_cloud(file)
        np_arr = np.asarray(pcd.points)  
        # print(pcd_file)
        np.savetxt(file.split('.pcd')[0]+".npy", np_arr)
        os.remove(file)



def main():
    parser = argparse.ArgumentParser()
    #parser.add_argument('--images_dir', required=True, type=str, help='images_folder')
    #parser.add_argument('--lidar_points_dir', required=True, type=str, help='lidar_points_folder')
    parser.add_argument('--output_dir', required = True, type = str, help = 'Output Directory')
    parser.add_argument('--bag_file', required=True, type=str, help='lidar_points_folder')
    #parser.add_argument('--camera_fps', required=True, type=float, help='camera frames per second')
    #parser.add_argument('--lidar_freq', required=True, type=float, help='lidar frequence in hertz')
    
    args = parser.parse_args()

    #output_dir = "/home/dheerajk/calibration_results"

    output_dir = args.output_dir
    # bag_file = extract_bag_file(output_dir, args.bag_file)
    # exit(0)
    
    bag_file = os.path.join(output_dir, args.bag_file)
    print("bag files : {}".format(bag_file))
    images_dir, lidar_points_dir =  extract_sensor_messages(output_dir, bag_file)
    images_dir = os.path.join(output_dir, "images")
    lidar_points_dir = os.path.join(output_dir, "lidar_points")
    
    synced_lidar_points_dir, synced_images_dir = timestamps(images_dir, lidar_points_dir, output_dir)
    # exit(0)
    synced_images_dir = os.path.join(output_dir, "synced_images")
    synced_lidar_points_dir = os.path.join(output_dir, "synced_lidar_points")
    extracted_lidar_points_dir, extracted_images_dir = image_select_gui(synced_images_dir, synced_lidar_points_dir, output_dir)    
    # exit(0)
    extracted_lidar_points_dir = os.path.join(output_dir, "extracted_lidar_points")
    extracted_images_dir = os.path.join(output_dir, "extracted_images")
    
    filtered_lidar_points_dir = os.path.join(output_dir,"filtered_pcd_files")
    filtered_images_dir = os.path.join(output_dir,"filtered_images")
    
    plots_dir = os.path.join(output_dir, "filtered_pcd_plots")
    # exit(0)
    # # print("extracted lidar points path : \n\n", sorted(os.listdir(extracted_lidar_points_dir)))
    # # # exit(0)
    if not os.path.exists(filtered_lidar_points_dir):
        os.mkdir(filtered_lidar_points_dir)

    if not os.path.exists(plots_dir):
        os.mkdir(plots_dir)
    

    if not os.path.exists(filtered_images_dir):
        os.mkdir(filtered_images_dir)
    
    filter_pcd_files(extracted_lidar_points_dir, extracted_images_dir, filtered_lidar_points_dir, filtered_images_dir, plots_dir)
    #exit(0)
    # extract_final_pair(extracted_images_dir, filtered_lidar_points_dir)
    # exit(0)
    # exit(0)
    pcd_to_numpy(filtered_lidar_points_dir)
    # exit(0)    
    # print(os.listdir(filtered_lidar_points_dir))
    # exit(0)
    training_dir = os.path.join(output_dir, "training")
    validation_dir = os.path.join(output_dir, "validation")
    random_sampling(filtered_images_dir, filtered_lidar_points_dir, training_dir, validation_dir)
    final_output_dir= os.path.join(output_dir, "results")
    # exit(0)
    calibration(training_dir, validation_dir, final_output_dir)

if __name__ == "__main__":	
    main()



    
