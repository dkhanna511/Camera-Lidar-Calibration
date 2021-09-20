
ROS_TOPICS= {"camera_topic" : "/camera/image_color", "lidar_topic": "/velodyne_points"}
#It is the angle in front of the lidar to filter out the lidar points in front of the calibration board. 
AZIMUTH_ANGLE= {"lower_val": -25, "upper_val": 25}
#range of distance in front of the lidar in which you want to filter out the lidar points of the calibration board 
DEPTH_THRESHOLD= {"lower_val" : 0, "upper_val": 5}
#number of images to save for training
NUM_SAMPLES= 25
CAMERA_FPS= 7
LIDAR_FREQ= 10
#length of each square on the calibration board
SQUARE_LENGTH = 0.108
#Boolean for plotting graph for lidar and camera normals.
PLOT_CAM_LIDAR_NORMALS = True
#duration for which the bag file has to be recorded.
DURATION = 120
