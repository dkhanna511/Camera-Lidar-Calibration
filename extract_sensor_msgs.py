# coding:utf-8
#!/usr/bin/python
    
# Extract images from a bag file.
    
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import pypcd
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os
import argparse
import yaml


with open ('calib_config.yaml') as f:
    config = yaml.full_load(f)

# Reading bag filename from command line or roslaunch parameter.
#import sys

# rgb_path = '/media/dheeraj/9A26F0CB26F0AA01/WORK/IIITD/Calibration/Calibration_Dataset/Dataset/images'
# lidar_path = '/media/dheeraj/9A26F0CB26F0AA01/WORK/IIITD/Calibration/Calibration_Dataset/Dataset/velodyne_points2'
# print(os.listdir(rgb_path))
class ImageCreator():
    
    
    def __init__(self, rgb_path_left, rgb_path_right, bag_file):
        self.bridge = CvBridge()
        with rosbag.Bag(bag_file,'r') as bag: #bag file to be read;
            for topic,msg,t in bag.read_messages():
                # print(topic)
                if topic == config["ros_topics"]["camera_topic_left"]: #topic of the image;
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # timstamp = rospy.Time.from_sec(timestr)
                    # timestr = msg.header.seq()
                    print(timestr)
                #                 #%.6f means there are 6 digits after the decimal point, which can be modified according to the accuracy requirements;
                    image_name = timestr+ ".png" #Image name: timestamp.png
                    cv2.imwrite(os.path.join(rgb_path_left , image_name), cv_image) #Save;
                
                
                if topic == config["ros_topics"]["camera_topic_right"]: #topic of the image;
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                    except CvBridgeError as e:
                        print(e)
                    timestr = "%.6f" % msg.header.stamp.to_sec()
                    # timstamp = rospy.Time.from_sec(timestr)
                    # timestr = msg.header.seq()
                    print(timestr)
                #                 #%.6f means there are 6 digits after the decimal point, which can be modified according to the accuracy requirements;
                    image_name = timestr+ ".png" #Image name: timestamp.png
                    cv2.imwrite(os.path.join(rgb_path_right , image_name), cv_image) #Save;
                
                
                # if topic == config["ros_topics"]["lidar_topic"]: #topic of the lidar;
                    
                #     pc = pypcd.PointCloud.from_msg(msg)
                #     # except CvBridgeError as e:
                #     #     print(e)
                #     timestr = "%.6f" % msg.header.stamp.to_sec()
                #     # print(timestr)
                #                 #%.6f means there are 6 digits after the decimal point, which can be modified according to the accuracy requirements;
                #     lidar_name = timestr+ ".pcd" #Image name: timestamp.png
                #     pc.save_pcd(os.path.join(lidar_path ,lidar_name)) #Save;
                
                

def extract_sensor_msges(output_dir, bag_file):


    if not os.path.exists(output_dir,):
        os.makedirs(os.path.join(output_dir, "framesLeft"))
        os.makedirs(os.path.join(output_dir, "framesRight"))
        os.makedirs(os.path.join(output_dir, "framesLidar"))
        
    rgb_path_left = os.path.join(output_dir, "framesLeft")
    rgb_path_right= os.path.join(output_dir, "framesRight")

    try:
        image_creator = ImageCreator(rgb_path_left, rgb_path_right, bag_file)
    except rospy.ROSInterruptException:
        pass

    return rgb_path_left, rgb_path_right

if __name__ == '__main__':
    
    #rospy.init_node(PKG)
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--output_dir', required=True, type=str, help='images_folder')
    
    # parser.add_argument('--lidar_points_dir', required=True, type=str, help='lidar_points_folder')
    parser.add_argument('--bag_file', required=True, type=str, help='lidar_points_folder')
    
    args = parser.parse_args()

    # lidar_path = args.lidar_points_dir
    bag_file = args.bag_file
    output_dir = args.output_dir
    extract_sensor_msges(output_dir, bag_file)
    
