#!/usr/bin/env python3

import rospy

import os
import csv
import time
import cv2
import numpy as np
import math 
from datetime import datetime

from geometry_msgs.msg import TransformStamped, PoseStamped
from mavros_msgs.srv import StreamRate
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy


bridge = CvBridge()

" Set resolution and frame rate for data collection "
# set1: 848x480, 60fps
# set2: 1280x720, 30fps
# https://dev.intelrealsense.com/docs/high-speed-capture-mode-of-intel-realsense-depth-camera-d435

resolution_set = 1
if resolution_set == 1: 
    resolution_x, resolution_y, frame_rate = 848, 480, 30 
if resolution_set == 2: 
    resolution_x, resolution_y, frame_rate = 1280, 720, 30 

class GazeboDataset:
    def __init__(self):
        rospy.Rate(10)

        self.time_now = datetime.now()

        self.folder = "multi_tool_ws/multitool_dataset/" +str(self.time_now)
        self.path = self.folder + "/"

        self.color_path = []
        self.depth_path = []
        n_cameras = 3
        for i in range(n_cameras):
            self.color_path.append(self.path + "cam_" + str(i) + "_" + str(resolution_x) + 'x' + str(resolution_y) + "x" + str(frame_rate) + '_rgb.avi')
            self.depth_path.append(self.path + "cam_" + str(i) + "_" + str(resolution_x) + 'x' + str(resolution_y) + "x" + str(frame_rate) + '_depth.avi')
        print(self.color_path)
        print(self.depth_path)
        
        self.poses_path = self.path + "_poses.csv"

        " Create csv "
        self.create_folder()

        self.writer_file = None
        with open(self.poses_path, 'w', newline='') as self.writer_file:
            self.writer_csv = csv.writer(self.writer_file)
            self.writer_csv.writerow(['type', 'frame', 'p_x', 'p_y', 'p_z', 'euller_x', 'euller_y', 'euller_z'])

        self.colorwriter_1 = cv2.VideoWriter(self.color_path[0], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)
        self.depthwriter_1 = cv2.VideoWriter(self.depth_path[0], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)

        self.colorwriter_2 = cv2.VideoWriter(self.color_path[1], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)
        self.depthwriter_2 = cv2.VideoWriter(self.depth_path[1], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)        
        
        self.colorwriter_3 = cv2.VideoWriter(self.color_path[2], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)
        self.depthwriter_3 = cv2.VideoWriter(self.depth_path[2], cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (resolution_x, resolution_y), True)

        self.start_d1= [3,0]
        self.pose_drone = ['drone',0,0,0,0,0,0,0]
        self.pose_camera = ['camera',0,0,0,0,0,0,0]
        self.frame_id = 0

        self.pose1 = None

        self.color_image_2 = None
        self.color_image_3 = None

        self.depth_image_1 = None
        self.depth_image_2 = None
        self.depth_image_3 = None

        rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, self.drone1_pose_local_callback)

        " Rostopic image "
        # rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_callback)
        # rospy.Subscriber("/ir/depth/image_raw_2", Image, self.depth_callback)

        rospy.Subscriber("/camera_1/image_raw_1", Image, self.image_callback) # main for data recording

        rospy.Subscriber("/camera_2/image_raw_2", Image, self.image_callback_2)
        rospy.Subscriber("/camera_3/image_raw_3", Image, self.image_callback_3)

        rospy.Subscriber("/camera_1/depth_1/image_raw", Image, self.depth_callback_1)
        rospy.Subscriber("/camera_2/depth_2/image_raw", Image, self.depth_callback_2)
        rospy.Subscriber("/camera_3/depth_3/image_raw", Image, self.depth_callback_3)
        

        # self.set_rate_client_1 = rospy.ServiceProxy("/drone1/mavros/set_stream_rate", StreamRate)
        # self.set_rate_client_2 = rospy.ServiceProxy("/drone2/mavros/set_stream_rate", StreamRate)
        # self.set_rate_client_1(stream_id=0, message_rate=100, on_off=True)
        # self.set_rate_client_2(stream_id=0, message_rate=100, on_off=True)
    

    def euler_from_quaternion(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)

        Args:
            x (_type_): _description_
            y (_type_): _description_
            z (_type_): _description_
            w (_type_): _description_

        Returns:
            roll_x, pitch_y, yaw_z: is rotation around x, y, z in radians (counterclockwise)
        """            

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1) 
        # roll_x = math.atan2(t0, t1) * (180 / math.pi)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        # pitch_y = math.asin(t2) * (180 / math.pi)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        # yaw_z = math.atan2(t3, t4)* (180 / math.pi)
        return roll_x, pitch_y, yaw_z # in radians

    def drone1_pose_local_callback(self, msg):
        self.pose1 = msg      
        euler = self.euler_from_quaternion(self.pose1.pose.orientation.x, self.pose1.pose.orientation.y, self.pose1.pose.orientation.z, self.pose1.pose.orientation.w) 
        self.pose_drone[0] = 'drone' 
        self.pose_drone[2] = self.pose1.pose.position.x
        self.pose_drone[3] = self.pose1.pose.position.y
        self.pose_drone[4] = self.pose1.pose.position.z
        self.pose_drone[5] = euler[0]
        self.pose_drone[6] = euler[2]
        self.pose_drone[7] = euler[2]

    def depth_callback_1(self, msg):
        cv_image_depth = bridge.imgmsg_to_cv2(msg, "32FC1")
        depth_image = np.asanyarray(cv_image_depth)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)
        # print(depth_image.shape, depth_image.max(), depth_image.min())
        self.depth_image_1 = depth_colormap

    def depth_callback_2(self, msg):
        cv_image_depth = bridge.imgmsg_to_cv2(msg, "32FC1")
        depth_image = np.asanyarray(cv_image_depth)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)
        # print(depth_image.shape, depth_image.max(), depth_image.min())
        self.depth_image_2 = depth_colormap
    
    def depth_callback_3(self, msg):
        cv_image_depth = bridge.imgmsg_to_cv2(msg, "32FC1")
        depth_image = np.asanyarray(cv_image_depth)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=1), cv2.COLORMAP_JET)
        # print(depth_image.shape, depth_image.max(), depth_image.min())
        self.depth_image_3 = depth_colormap

    def image_callback_2(self, msg):
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image_2 = np.asanyarray(cv_img)

    def image_callback_3(self, msg):
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image_3 = np.asanyarray(cv_img)

    def image_callback(self, msg):
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        color_image = np.asanyarray(cv2_img)
        cv2.imshow('Stream', color_image)

        self.frame_id += 1
        self.pose_drone[1] = self.frame_id 

        with open(self.poses_path, 'a', newline='') as file:
            self.writer_file = csv.writer(file)
            self.writer_file.writerow(self.pose_drone)

        print("colected frames:", self.frame_id)

        self.colorwriter_1.write(color_image)
        self.colorwriter_2.write(self.color_image_2)
        self.colorwriter_3.write(self.color_image_3)
        
        self.depthwriter_1.write(self.depth_image_1)
        self.depthwriter_2.write(self.depth_image_2)
        self.depthwriter_3.write(self.depth_image_3)
        
        if cv2.waitKey(1) == ord('q'):
            self.colorwriter_1.release()
            self.colorwriter_2.release()
            self.colorwriter_3.release()

            self.depthwriter_1.release()
            self.depthwriter_2.release()
            self.depthwriter_3.release()

            time.sleep(0.1)
            print('Saved to', self.path)

            cv2.destroyAllWindows()
            rospy.loginfo("exit()!")
            rospy.signal_shutdown("exit()!")

    def myhook(self):
        print("shutdown time!")

    def create_folder(self):
        " Create folder to store data set "
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            print("Directory created successfully!")
        else:
            print("Directory already exists!") 
        
if __name__ == '__main__':
    rospy.init_node('data_collection', anonymous=True)

    node = GazeboDataset()
    
    while not rospy.is_shutdown():
        pass
