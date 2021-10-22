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
import extract_sensor_msgs
import filter_pcd
import calibration
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



# def extract_sensor_messages(output_dir, bag_file):
#     # print(images_dir, lidar_points_dir, bag_file)
#     # exit(0)
#     images_dir = os.path.join(output_dir, "images")
#     lidar_points_dir = os.path.join(output_dir, "lidar_points")
#     # print(images_dir, lidar_points_dir, bag_file)
    
#     call_string = "python3 extract_sensor_msgs.py --images_dir {} --lidar_points_dir {} --bag_file {}".format(images_dir,lidar_points_dir,bag_file)
    
#     print(call_string)
   
#     # exit(0)
#     subprocess.call(call_string, shell = True)
#     call_string = "rosrun pcl_ros bag_to_pcd {} {} {}".format(bag_file, config["ros_topics"]["lidar_topic"], lidar_points_dir)
#     subprocess.call(call_string, shell = True)
    
#     return images_dir, lidar_points_dir
    
# def timestamps(images_dir, lidar_points_dir, output_dir):
#     call_string = "python3 timestamps.py --images_dir {} --lidar_points_dir {} --output_dir {}".format(images_dir,lidar_points_dir, output_dir)
#     print(call_string)
#     subprocess.call(call_string, shell = True)
#     # print("camera fps : ", camera_fps)
#     # print("lidar frequence : ", lidar_freq)
#     # if camera_fps > lidar_freq:
#     #     print("hello world 2")
#     #     synced_images_dir = os.path.join(output_dir, "synced_images")
#     #     synced_lidar_points_dir = os.path.join(output_dir, "lidar_points")
#     # elif camera_fps <  lidar_freq:
#     #     print("hello world")
#     #     synced_images_dir = os.path.join(output_dir, "images")
#     #     synced_lidar_points_dir = os.path.join(output_dir, "synced_lidar_points")
#     synced_images_dir = os.path.join(output_dir, "synced_images")
#     synced_lidar_points_dir = os.path.join(output_dir, "synced_lidar_points")

#     return synced_lidar_points_dir, synced_images_dir
    

# def image_select_gui(images_dir, lidar_points_dir, output_dir):

#     call_string = "python3 image_select_gui.py --images_dir {} --lidar_points_dir {} --output_dir {}".format(images_dir,lidar_points_dir, output_dir)
#     print(call_string)
#     subprocess.call(call_string, shell = True)
    
#     lidar_points_dir = os.path.join(output_dir, "extracted_lidar_points")
#     images_dir = os.path.join(output_dir, "extracted_images")

#     return lidar_points_dir, images_dir



# def filter_pcd_files(lidar_points_dir, images_dir, filtered_lidar_points_dir, filtered_images_dir, plots_dir):

#     call_string = "python3 filter_pcd.py --lidar_points_dir {} --images_dir {} --output_dir_lidar {} --output_dir_images {} --plots_dir {}".format(lidar_points_dir,images_dir, filtered_lidar_points_dir,filtered_images_dir ,plots_dir)
#     print(call_string)
#     subprocess.call(call_string, shell = True)


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


def randomSamplingCalibration(imagesDirLeft, imagesDirRight, lidarPointsDir, trainingDir, validationDir):
    filenames = ["imagesLeft", "imagesRight","lidarPoints"]

    for file in filenames:
        if not os.path.exists(os.path.join(trainingDir,file)):
            os.makedirs(os.path.join(trainingDir, file))
            os.makedirs(os.path.join(validationDir, file))
    
    
    num_train = random.sample(range(len(os.listdir(imagesDirLeft))), config["num_samples_train"])
    print(len(num_train))
    # print(len(images_dir))
    print(num_train)
    # print(os.listdir(images_dir))
    for index, (imageLeft, imageRight, lidarPoints) in enumerate(zip(sorted(os.listdir(imagesDirLeft)),sorted(os.listdir(imagesDirRight)), sorted(os.listdir(lidarPointsDir)))):
        fileImageLeft = os.path.join(imagesDirLeft, imageLeft)
        fileImageRight = os.path.join(imagesDirRight, imageRight)
        
        fileLidar = os.path.join(lidarPointsDir, lidarPoints)
        if index in num_train:
            shutil.copy(fileImageLeft, os.path.join(trainingDir, filenames[0], imageLeft))
            shutil.copy(fileImageRight, os.path.join(trainingDir, filenames[1], imageRight))
            shutil.copy(fileLidar, os.path.join(trainingDir,filenames[2], lidarPoints))

        else:
            shutil.copy(fileImageLeft, os.path.join(validationDir,filenames[0], imageLeft))
            shutil.copy(fileImageRight, os.path.join(validationDir,filenames[1], imageRight))
            shutil.copy(fileLidar, os.path.join(validationDir,filenames[2], lidarPoints))


# def calibration(trainingDir, testingDir, outputDir):
#     if not os.path.exists(outputDir):
#         os.makedirs(outputDir)
#     print("##########Calibration results for training##########\n")
#     call_string_train = "python3 calibration.py --trainingDir {} --output_dir {}".format(trainingDir, outputDir)
    
#     subprocess.call(call_string_train, shell = True)
#     print("##########Calibration results for testing##########\n")
#     call_string_test = "python3 calibration.py --testing_dir {} --output_dir {}".format(testingDir, outputDir)
#     subprocess.call(call_string_test, shell = True)


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




def randomSampling(outputDir, leftImagesDir, rightImagesDir, lidarPointsDir):
    filenames = ["images", "lidar_points"]
    if not os.path.exists(os.path.join(outputDir, "extractedFramesLeft")):
        os.makedirs(os.path.join(outputDir, "extractedFramesLeft"))
        os.makedirs(os.path.join(outputDir, "extractedFramesRight"))
        os.makedirs(os.path.join(outputDir, "extractedFramesLidar"))
        
    extractedImagesLeft = os.path.join(outputDir, "extractedFramesLeft")
    extractedImagesRight= os.path.join(outputDir, "extractedFramesRight")
    extractedLidarPoints= os.path.join(outputDir, "extractedFramesLidar")

    
    num_train = random.sample(range(len(os.listdir(leftImagesDir))), 50)
    print(len(num_train))
    # print(len(images_dir))
    print(num_train)
    # print()
    # print(os.listdir(images_dir))
    for index, (imagesLeft,imagesRight, lidarPoints) in enumerate(zip(sorted(os.listdir(leftImagesDir)), sorted(os.listdir(rightImagesDir)), sorted(os.listdir(lidarPointsDir)))):
        fileImageLeft = os.path.join(leftImagesDir, imagesLeft)
        print(fileImageLeft)
        print(imagesLeft)
        print(index)
        fileImageRight = os.path.join(rightImagesDir, imagesRight)
        
        fileLidarPoints = os.path.join(lidarPointsDir, lidarPoints)
        if index in num_train:
            shutil.copy(fileImageLeft, os.path.join(extractedImagesLeft, imagesLeft))
            shutil.copy(fileImageRight, os.path.join(extractedImagesRight, imagesRight))
            shutil.copy(fileLidarPoints, os.path.join(extractedLidarPoints, lidarPoints))

    return extractedImagesLeft, extractedImagesRight, extractedLidarPoints


# def camToLidar(camImages, lidarPoints)


def main():
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--output_dir', required = True, type = str, help = 'Output Directory')
    parser.add_argument('--bag_file', required=True, type=str, help='lidar_points_folder')
    
    args = parser.parse_args()

    outputDir = args.output_dir
    # bag_file = extract_bag_file(output_dir, args.bag_file)
    
    # bagFile = os.path.join(outputDir, args.bag_file)
    bagFile = args.bag_file
    
    print("bag files : {}".format(bagFile))
    
    lidarPointsDir = os.path.join(outputDir, "framesLidar")
    leftImagesDir = os.path.join(outputDir, "framesLeft")
    rightImagesDir = os.path.join(outputDir, "framesRight")
    
    extractedLidarPointsDir = os.path.join(outputDir, "extractedFramesLidar")
    extractedLeftImagesDir = os.path.join(outputDir, "extractedFramesLeft")
    extractedRightImagesDir = os.path.join(outputDir, "extractedFramesRight")
    
    
    if not os.path.exists(extractedLidarPointsDir):
        os.makedirs(extractedLidarPointsDir)

    if not os.path.exists(extractedLeftImagesDir):
        os.makedirs(extractedLeftImagesDir)
    
    if not os.path.exists(extractedRightImagesDir):
        os.makedirs(extractedRightImagesDir)
    
    # leftImagesDir, rightImagesDir= extract_sensor_msgs.extract_sensor_msges(outputDir, bagFile)
    
    print(leftImagesDir)
    print(os.listdir(leftImagesDir))
    
    
    if len(os.listdir(extractedLeftImagesDir)) ==0:
        extractedLeftImagesDir, extractedRightImagesDir, extractedLidarPointsDir = randomSampling(outputDir, leftImagesDir, rightImagesDir,lidarPointsDir)
    # print(extractedImagesLeft, extractedImagesDir, extractedLidarPointsDir)
    
    filteredLidarPointsDir = os.path.join(outputDir,"filteredPcdFiles")
    filteredLeftImagesDir = os.path.join(outputDir,"filteredLeftImages")
    filteredRightImagesDir = os.path.join(outputDir,"filteredRightImages")
    
    plotsDir = os.path.join(outputDir, "filteredPcdPlots")
    # exit(0)
    # print("extracted lidar points path : \n\n", sorted(os.listdir(extracted_lidar_points_dir)))
    # # # exit(0)
    if not os.path.exists(filteredLidarPointsDir):
        os.makedirs(filteredLidarPointsDir)

    if not os.path.exists(plotsDir):
        os.makedirs(plotsDir)
    
    if not os.path.exists(filteredLeftImagesDir):
        os.makedirs(filteredLeftImagesDir)
    

    if not os.path.exists(filteredRightImagesDir):

        os.makedirs(filteredRightImagesDir)
    
    if len(os.listdir(filteredLeftImagesDir)) ==0:
        filter_pcd.filter_pcd(extractedLidarPointsDir, extractedLeftImagesDir, extractedRightImagesDir, filteredLidarPointsDir,  filteredLeftImagesDir, filteredRightImagesDir, plotsDir )
    
    pcd_to_numpy(filteredLidarPointsDir)
    
    
    trainingDir = os.path.join(outputDir, "training")
    print(trainingDir)
    validationDir = os.path.join(outputDir, "validation")
    # print(filteredImagesDir)lidar_points_dir
    if not os.path.exists(trainingDir):
        os.makedirs(trainingDir)
    if not os.path.exists(validationDir):
        os.makedirs(validationDir)

    if len(os.listdir(trainingDir))==0:
        randomSamplingCalibration(filteredLeftImagesDir, filteredRightImagesDir, filteredLidarPointsDir, trainingDir, validationDir)
    
    # exit(0)
    finalOutputDir= os.path.join(outputDir, "results")

    calibration.stereoCalibration(trainingDir, validationDir, finalOutputDir)
    calibration.camLidarCalibration(trainingDir, validationDir, outputDir, camera = "left")
    calibration.camLidarCalibration(trainingDir, validationDir, outputDir, camera = "right")

if __name__ == "__main__":	
    main()



    
