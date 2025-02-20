#!/usr/bin/env python3
import rospy
import numpy as np
import random
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
import cv2
from spline_traj import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import csv
from datetime import datetime
import os

class SpawnerCamera:
    def __init__(self, init_point = [0,0,0], goal_pooints=[]):
        # rospy.init_node('move_model_to_goal')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Начальная и конечная позиция
        self.init_pose = init_point
        self.goal_poses = goal_pooints
        
        self.init_pose_rand = [0,0,0]
        self.init_noised_distance = 1.0
        self.set_init_pose()

        self.fixed_z = 0.75

        self.frame_id = 0
        self.bridge = CvBridge()
        self.color_image = None
        self.writer_file = None
        rospy.Subscriber("/camera_1/image_raw_1", Image, self.image_callback) # main for data recording

    def create_folder(self):
        " Create folder to store data set "
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)
            print("Directory created successfully!")
        else:
            print("Directory already exists!") 

        if not os.path.exists(self.folder+"/images"):
            os.makedirs(self.folder+"/images")
        else:
            print("Directory already exists!") 

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.color_image = np.asanyarray(cv2_img)

    def save_image(self, filename: str):
        if not os.path.exists(self.folder+"/images"):
            os.makedirs(self.folder+"/images")
        filepath = os.path.join(self.folder+"/images", filename)
        cv2.imwrite(filepath, self.color_image)

    def set_init_pose(self):
        state_msg = ModelState()
        state_msg.model_name = 'iris_demo'  # Имя модели
        state_msg.pose.position.x = self.init_pose[0]
        state_msg.pose.position.y = self.init_pose[1]
        state_msg.pose.position.z = self.init_pose[2]
        self.set_state(state_msg)

    def init_folders_to_store_data(self):
        self.folder = "/home/sim/ardupilot_docker/dataset/" +str(datetime.now()) + "/"
        self.create_folder()
        with open(self.folder + "data.csv", 'w', newline='') as self.writer_file:
            self.writer_csv = csv.writer(self.writer_file)
            self.writer_csv.writerow(['frame_id', 'dx', 'dy', 'dz','angle', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def move_camera(self): 
        for goal_pose in self.goal_poses:

            self.init_folders_to_store_data()
            self.frame_id = 0

            self.init_pose_rand[0] = self.init_pose[0] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            self.init_pose_rand[1] = self.init_pose[1] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            self.init_pose_rand[2] = self.init_pose[2] + random.uniform(-self.init_noised_distance, self.init_noised_distance)
            
            if self.init_pose_rand[2] <= 0.2:
                self.init_pose_rand[2] = 0.2

            x_vals,y_vals = main_2d(self.init_pose_rand, goal_pose)
            z_vals = np.linspace(self.init_pose_rand[2], self.fixed_z, len(x_vals))

            # Диапазоны шума
            position_noise_range = 0.001  # Максимальный шум для позиции (0.2 м)
            orientation_noise_range = 0.005  # Максимальный шум для ориентации (0.1 рад)

            # Начинаем движение по траектории
            for i in range(len(x_vals)-1):
                # Позиция на текущем шаге с шумом
                x = x_vals[i] + random.uniform(-position_noise_range, position_noise_range)
                y = y_vals[i] + random.uniform(-position_noise_range, position_noise_range)
                z = z_vals[i] + random.uniform(-position_noise_range, position_noise_range)

                # Рассчитаем углы для ориентации
                dx = x_vals[i] - x_vals[i+1]
                dy = y_vals[1] - y_vals[1+1]
                dz = z_vals[i] - z_vals[i+1]
                angle = np.arctan2(dy, -dx)  # угол для ориентации
                    
                # Ориентация с шумом
                noise_roll = random.uniform(-orientation_noise_range, orientation_noise_range)
                noise_pitch = random.uniform(-orientation_noise_range, orientation_noise_range)
                noise_yaw = random.uniform(-orientation_noise_range, orientation_noise_range)

                # Новый угол ориентации с учётом шума
                noisy_angle = angle + noise_yaw
                
                # Преобразуем шум и угол в кватернион
                quat = quaternion_from_euler(noise_roll, noise_pitch, noisy_angle * 5)
                
                # Создание сообщения о состоянии модели
                state_msg = ModelState()
                state_msg.model_name = 'iris_demo'  # Имя модели
                state_msg.pose.position.x = x
                state_msg.pose.position.y = y
                state_msg.pose.position.z = z
                state_msg.pose.orientation.x = quat[0]
                state_msg.pose.orientation.y = quat[1]
                state_msg.pose.orientation.z = quat[2]
                state_msg.pose.orientation.w = quat[3]
                
                # Отправка команды в Gazebo
                self.set_state(state_msg)

                # Save trajectories
                if self.frame_id > 0: #FIXME: if frame id == 0 image came from last of the previuse flight.
                    with open(self.folder + 'data.csv', 'a', newline='') as file:
                        self.writer_file = csv.writer(file)
                        self.writer_file.writerow([self.frame_id,dx,dy,dz,angle,x,y,z,angle,quat[0],quat[1],quat[2],quat[3]])

                    # Save images
                    self.save_image('image_'+str(self.frame_id)+'.jpg')

                rospy.sleep(0.02)  # Пауза между шагами
                self.frame_id += 1

            rospy.loginfo("Model reached goal position.")

        self.set_init_pose()
