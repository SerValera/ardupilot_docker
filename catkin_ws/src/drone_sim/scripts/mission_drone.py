#!/usr/bin/env python3

import rospy
from drone_autopilot import Drone
from std_msgs.msg import String, Time, Bool
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import Waypoint
from datetime import datetime

from geometry_msgs.msg import PoseStamped

import os
import time
import math
import numpy as np
import csv
import json

import threading

from mavros_msgs.msg import RCIn

class DroneMission:
    def __init__(self):
        self.drone = Drone()
        self.drone.id = 0
        self.is_gazebo_sim = True # True to test in gazebo, false for working with real UAV.

        self.is_log_coord = False
        
        self.filename_log = 'mission_log'
        self.filename_task_list = 'task_list.json'
        self.filename_log_cords = 'mission_cords.csv'
        self.is_start_record_cords = False
        self.is_start_demo_mission = False
        self.file_name_log_cords = None
        self.start_time = None
        self.on_mission = False

        if self.is_gazebo_sim:
            " Data collection laptop Valerii "
            self.pathname_log = "/home/vs/ardupilot_ws/src/ardupilot_sim/results/"
            " WSL Ubunty valerii laptop "
            self.pathname_task_list = "/home/vs/brain_ws/src/drone-master/ardupilot_control/task_lists/"
            # self.pathname_log = "/home/vs/brain_ws/src/drone-master/ardupilot_control/mission_logs/"
            " ASAD workstation "
            # self.pathname_task_list = "/home/asad/uav-flight-simulation/catkin_ws/src/ardupilot_control/scripts/mission/"
            # self.pathname_log = "/home/asad/uav-flight-simulation/catkin_ws/src/ardupilot_control/mission_logs/"

            self.is_check_ten_degree = False
            self.is_check_small_move = False
            self.test_real_flight = True
            self.is_demo_flight_remote = False

        else:
            self.pathname_task_list = "/home/pi/catkin_ws/src/drone/ardupilot_control/task_lists/"
            self.pathname_log = "/home/pi/catkin_ws/src/drone/ardupilot_control/mission_logs/"
            self.is_check_ten_degree = rospy.get_param('/is_check_ten_degree')
            self.is_check_small_move = rospy.get_param('/is_check_small_move')
            self.test_real_flight = rospy.get_param('/test_real_flight') # works for real and gazebo drones
            self.is_demo_flight_remote = rospy.get_param('/is_demo_flight_remote')
            self.drone.name = rospy.get_param('/UAV_name')

        
        print("UAV_name:", self.drone.name)

        " Path for drone in local "
        self.start_d1= [3,0]
        self.trajectory_type = int(rospy.get_param('/trajectory_type'))

        self.path_test = []
        if self.trajectory_type == 1: # cyrcle
            self.generate_cyrcle()
        if self.trajectory_type == 2: # square
            self.path_test = [
                [0, 3, 3], 
                [-1, 3.25, 3], 
                [-2, 3.5, 3], 
                [-3, 3.75, 3], 
                [-4, 4, 3], 
                [-5, 4.25, 3], 
                [-6, 4.5, 3], 
                [-7, 4.75, 3], 
                [-8, 5, 3], 
                [-9, 5.25, 3], 
                [-10, 5.5, 3], 
                [-11, 5.75, 3], 
                [-12, 6, 3], 
                [-13, 6.25, 3], 
                [-14, 6.5, 3], 
                [-15, 6.75, 3], 
                [-16, 7, 3], 
                [-17, 7.25, 3], 
                [-18, 7.5, 3], 
                [-19, 7.75, 3], 
                [-20, 8, 3],
                [-10, 8, 3]
                ]

        self.is_flight_circle_started = False
        self.is_follow_aruco = False
        self.is_start_overtake = False
        self.overtake_path = []
        
        " Set drone coefitiont of speed "
        if self.drone.name == "/drone1":
            self.drone.k_vel = rospy.get_param('/speed_main')

        if self.drone.name == "/drone2":
            self.drone.k_vel = rospy.get_param('/speed_aruco')

        " Is collect data set "
        self.is_collect_data_set = rospy.get_param("/is_collect_data_set")
        self.r_dataset = 3.0
        self.z_dataset = 3.0
        self.point_dataset = [-5.0, 5.0]
        self.drone.yaw_correction = 0.0
        self.drone.yaw_dataset = []
        
        if self.is_collect_data_set:
            self.generate_cyrcle_dataset()
            self.drone.yaw_correction = 0.0

        " Command variables "
        self.id_task_prev = None
        self.is_command_new = True

        " Mission executuion topic "
        rospy.Subscriber('/drone_commands_gs', Waypoint, self.callback_commands) #from ground_station.py
        self.rate = rospy.Rate(10)

        rospy.Subscriber(self.drone.name + "/mavros/imu/data", Imu, self.check_for_movement) 

        print("Create folder to record data")
        self.time_start = None

        " Create lofing file for coordinates "
        self.file_path_data_recording = None       

    ### --- CALLBACK FUNCTIONS ---
    def drone2_pose_local_callback(self, msg):
        pose_local = msg

    ### --- EMERGENCY LAND ---
    def end_mission_drone(self):
        self.drone.land()

    def callback_commands(self, msg):
        """ Callback function for getting data and commands form ground_station.py
        from topic '/drone_commands_gs' 

        Args:
            msg (Waypoint): message in Waypoint format.
        """        
        param_command = msg
        id_task = param_command.command
        rospy.loginfo('Get id task: ' + str(id_task))

        if self.id_task_prev != id_task:
            self.id_task_prev = id_task
            self.is_command_new = True

        if self.is_command_new == True:
            if id_task == 0:
                print("WARNING OCCURIED")
                # TODO: WARNING
                
            if id_task == 1:
                print("TEST vel_publisher")
                self.drone.pub_vel([1.0, 0.0, 0.0], yaw_rot=0)

            if id_task == 2:
                print("Start_record_cords")
                self.is_start_record_cords = True
                self.time_start = time.time()
            
            if id_task == 3:
                print("Stop_record_cords")
                self.is_start_record_cords = False
       

            """ Drone control test commands """
            if id_task == 11: #arm
                self.drone.arm()
            if id_task == 12: #disarm
                self.drone.dis_arm()
            if id_task == 13: #takeoff
                
                # logging coordinates TEST
                self.start_time = time.time()
                self.start_data = datetime.today().strftime('%Y_%m_%d_%H_%M_%S')
                self.create_log_cords_file(self.start_data) # create cords liging file
                # self.is_start_record_cords = True #allow to record in timer callback

                attitude = float(param_command.z_alt)
                self.drone.arm()
                self.drone.takeoff(height=attitude)

            if id_task == 14: #land
                self.drone.land()
                self.is_start_record_cords = False
            if id_task == 15: #set home
                self.drone.set_new_home_pos()
            if id_task == 16: #RTL mode
                self.drone.set_mode("RTL")

            if id_task == 22: #demo flight
                rospy.loginfo('---start_demo_flight_from_remote---')
                self.is_start_demo_mission = True
                # self.demo_flight_mission()

            if id_task == 20:  # 'Start local mission'
                # Setting mission parameters from terminal. Tested!!! Check YAW
                " get data from mission "
                steps = int(param_command.param1)
                d_time = int(param_command.param2)
                delta_x = float(param_command.x_lat)
                alt = float(param_command.z_alt)

                self.drone.set_new_home_pos()
                time.sleep(2)

                " get yaw in grad "
                angle = self.drone.orientation_grad[2]

                " generate points "
                point = []
                for x in range(steps):
                    point.append([(x+1) * delta_x * math.cos(angle * math.pi/180)+self.drone.local_home_pos[0],
                        (x+1) * delta_x * math.sin(angle * math.pi/180) + self.drone.local_home_pos[1], alt])

                " start flight point to point "
                for x in range(steps):
                    rospy.loginfo('MISSION POINT: ' + str(x))
                    self.drone.arm()
                    rospy.loginfo(self.drone.current_status)
                    while self.drone.current_status == 'Arming':
                        pass
                    rospy.loginfo(self.drone.current_status)
                    self.drone.takeoff(height=alt)
                    while self.drone.current_status == "TakeOff":
                        pass
                    rospy.loginfo(self.drone.current_status)
                    self.drone.goTo(point[x], mode='global', yaw=angle)
                    rospy.loginfo(self.drone.current_status)
                    while self.drone.current_status == "Going_to_local":
                        pass
                    self.drone.land()
                    rospy.loginfo(self.drone.current_status)
                    while self.drone.current_status == "Landing":
                        pass
                    time.sleep(1)

                " retern to home position "
                self.drone.arm()
                while self.drone.current_status == 'Arming':
                    pass
                self.drone.takeoff(height=alt)
                while self.drone.current_status == "TakeOff":
                    pass
                self.drone.goTo([0,0,alt], mode='global', yaw=angle)
                rospy.loginfo(self.drone.current_status)
                while self.drone.current_status == "Going_to_local":
                    pass
                self.drone.land()
                rospy.loginfo(self.drone.current_status)
                while self.drone.current_status == "Landing":
                    pass
                rospy.loginfo(self.drone.current_status)
                rospy.loginfo("MISSION END")
            
            if id_task == 21:  # 'Go to local pos (in flight!)
                m = int(param_command.param1)
                point = [float(param_command.x_lat), float(
                    param_command.y_long), float(param_command.z_alt)]
                yaw_input = float(param_command.param2)
                self.drone.goTo(point, mode=m, yaw=yaw_input)


            # list_waypoints_1 = [[3,0,3,180], [3,5,3,90], [8,5,3,0], [8,0,3,270]]
            list_waypoints = [[0,0,3,180], [0,5,3,90], [5,5,3,0], [5,0,3,270]]
            

            if id_task == 51:
                if self.drone.name == "/drone1":
                    print('Task for drone 1. Main')
                    for point in list_waypoints:
                        self.start_d1
                        self.drone.goTo([point[0] + self.start_d1[0],point[1] + self.start_d1[1],point[2]], mode="global")

            if id_task == 52:
                if self.drone.name == "/drone2":
                    print('Task for drone 2. Aruko')
                    for point in list_waypoints:
                        self.drone.goTo([point[0],point[1],point[2]], mode="global", yaw=point[3])

            if id_task == 53:
                if self.drone.name == "/drone1":
                    for point in self.path_test:
                        self.drone.goTo([point[0] + self.start_d1[0],point[1] + self.start_d1[1],point[2]], mode="global")

            " Velocity cyrcle drone 1 main"
            if id_task == 54:
                if self.drone.name == "/drone1":
                    self.is_follow_aruco = False
                    self.is_flight_circle_started = True
                    self.is_start_overtake = False
                    self.drone.k_vel = rospy.get_param('/speed_main')
                    self.drone.path_len = len(self.path_test)

            " Velocity cyrcle drone 2 aruco"
            if id_task == 55:
                if self.drone.name == "/drone2":
                    self.is_flight_circle_started = True
                    self.drone.path_len = len(self.path_test)
                    print(self.is_flight_circle_started)

            " Velocity cyrcle drone 1 main"
            if id_task == 61:
                if self.drone.name == "/drone1":
                    self.is_follow_aruco = True
                    self.is_flight_circle_started = False
                    self.is_start_overtake = False
                    
                    self.drone.k_vel = rospy.get_param('/speed_aruco')
                    self.drone.path_len = len(self.path_test)

            if id_task == 70:
                if self.drone.name == "/drone2":
                    self.drone.goTo([self.point_dataset[0], self.point_dataset[1], 5.0], yaw=90.0)

            if id_task == 71:  # 'Start local mission'
                # Setting mission parameters from terminal. Tested!!! Check YAW
                " get data from mission "
                self.z_dataset = int(param_command.param1)
                self.r_dataset = int(param_command.param2)
                self.drone.overtake_counter = 0
                self.generate_cyrcle_dataset()

    def check_for_movement(self, data):
        def euler_from_quaternion(x, y, z, w):
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

        """ Get data from IMU """
        w = float(data.orientation.w)
        x = float(data.orientation.x)
        y = float(data.orientation.y)
        z = float(data.orientation.z)

        """ Convert imu data from quaternion to euler in radian """
        roll_x_e, pitch_y_e, yaw_z_e = euler_from_quaternion(x, y, z, w)
        roll_x, pitch_y, yaw_z = [round(roll_x_e * 180 / math.pi, 2), round(pitch_y_e * 180 / math.pi, 2), round(yaw_z_e * 180 / math.pi, 2)]


    def on_mission_local_time(self):
        return round(time.time() - self.start_time, 2)

        
    def calc_dists_global(self, p1, p2):
        """Calculate distance between two geo positions 

        Args:
            p1 ([float, float]): geoposition 1 [lat1, lon1]
            p2 ([float, float]): geoposition 2 [lat2, lon2]

        Returns:
            distance [float]: distance in meters 
        """    
        R = 6378137.0 # approximate radius of earth in meters
        lat1, lon1 = math.radians(float(p1[0])), math.radians(float(p1[1]))
        lat2, lon2 = math.radians(float(p2[0])), math.radians(float(p2[1]))
        a = math.sin((lat2 - lat1) / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return round(R * c, 2)

    def create_log_cords_file(self, data_time):
        " cord logs"
        if self.is_log_coord:
            folder_name = 'mission_' + data_time
            if not os.path.isdir(self.pathname_log + folder_name +'/'):
                os.mkdir(self.pathname_log + folder_name +'/')
            header = ['time', 'lat', 'lon', 'alt', 'x', 'y', 'z']
            self.file_name_log_cords = self.pathname_log + folder_name + '/' + self.filename_log_cords
            with open(self.file_name_log_cords, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)

    def generate_cyrcle_dataset(self):
        #The lower this value the higher quality the circle is with more points generated
        stepSize = math.pi * 2 / 50
        #Generated vertices
        positions = []
        yaw = []
        t = 0
        while t < 2 * math.pi:
            positions.append([self.r_dataset * math.cos(t) + self.point_dataset[0], self.r_dataset * math.sin(t) + self.point_dataset[1], self.z_dataset])
            t += stepSize
            yaw.append(t)
        self.path_test = positions
        self.drone.yaw_dataset = yaw
        self.drone.path_len = len(positions)

    def generate_cyrcle(self):
        a = -10
        b = 8
        r = 7.5
        #The lower this value the higher quality the circle is with more points generated
        stepSize = math.pi * 2 / 20
        #Generated vertices
        positions = []
        t = 0
        while t < 2 * math.pi:
            positions.append([r * math.cos(t) + a, r * math.sin(t) + b, 3.0])
            t += stepSize
        self.path_test = positions
        self.drone.path_len = len(positions)

    def demo_flight_mission(self, event=None):
        # self.is_follow_aruco = False
        # self.is_flight_circle_started = False
        # self.is_start_overtake = True
    
        " Cyrcular path "
        if self.is_flight_circle_started:
            path_goal_id = self.drone.path_counter % self.drone.path_len
            goal_p = self.path_test[path_goal_id]

            if self.drone.name == "/drone1":
                self.drone.goToVel([goal_p[0] + self.start_d1[0], goal_p[1] + self.start_d1[1], goal_p[2]])

            if self.drone.name == "/drone2":
                self.drone.goToVel([goal_p[0], goal_p[1], goal_p[2]])
                self.drone.publish_id_path.publish(str(path_goal_id))

        " Overtaking "
        if self.is_start_overtake:
            if self.drone.name == "/drone1":
                if self.drone.overtake_counter < self.drone.overtake_len:
                    goal_p_over = self.overtake_path[self.drone.overtake_counter]
                    self.drone.goToVel_overtake([goal_p_over[0], goal_p_over[1], goal_p_over[2]])
                else: 
                    self.is_flight_circle_started = True
                    self.is_start_overtake = False
           


if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)

    mission = DroneMission()

    rospy.Timer(rospy.Duration(0.1), mission.demo_flight_mission)

    while not rospy.is_shutdown():
        # mission.demo_flight_mission()
        rospy.spin()
        
