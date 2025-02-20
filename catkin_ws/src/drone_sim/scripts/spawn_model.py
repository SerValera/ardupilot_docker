#!/usr/bin/env python3

import rospy
import csv
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import tf.transformations as tf

from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations as tf

from camera_move import SpawnerCamera

SDF_PATH = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/"
CSV_FILE = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/scene_setup.csv"

class SpawnerModels:
    def __init__(self):
        self.spawned_models = set()

        self.init_pose = [-1, 0, 0.75]
        self.goal_points = []


    def delete_existing_models(self):
        rospy.wait_for_service("/gazebo/delete_model")
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        
        for model in list(self.spawned_models):
            try:
                delete_model(model)
                rospy.loginfo(f"Deleted {model}")
                self.spawned_models.remove(model)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to delete {model}: {e}")

    def spawn_label(self, model_name, x, y, z):
        model_sdf_file = os.path.join(SDF_PATH, "label", "model.sdf")
        if not os.path.exists(model_sdf_file):
            rospy.logerr(f"Model SDF not found: {model_sdf_file}")
            return
        with open(model_sdf_file, "r") as f:
            model_xml = f.read()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z + 2
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        name = model_name+'_label'
        try:
            spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_service(name, model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {name} at ({x}, {y}, {z})")
            self.pawned_models.add(name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {name}: {e}")


    def rotate_model(self, id, model_name, roll, pitch, yaw):
        gazebo_model_name = model_name + "_" + str(id)
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            current_state = get_state(gazebo_model_name, "world")

            if current_state.success:
                quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
                state = ModelState()
                state.model_name = gazebo_model_name
                state.pose.position = current_state.pose.position  # Keep original position
                state.pose.orientation.x = quaternion[0]
                state.pose.orientation.y = quaternion[1]
                state.pose.orientation.z = quaternion[2]
                state.pose.orientation.w = quaternion[3]
                resp = set_state(state)
                return resp
            else:
                print(f"Failed to get state for model {gazebo_model_name}")

        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def spawn_model(self, id, model_name, x, y, z):
        model_sdf_file = os.path.join(SDF_PATH, model_name, "model.sdf")
        
        if not os.path.exists(model_sdf_file):
            rospy.logerr(f"Model SDF not found: {model_sdf_file}")
            return

        with open(model_sdf_file, "r") as f:
            model_xml = f.read()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        gazebo_model_name = model_name + "_" + str(id)
        try:
            spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_service(gazebo_model_name, model_xml, "", pose, "world")
            rospy.loginfo(f"Spawned {model_name} with id {id} at ({x}, {y}, {z})")
            self.spawned_models.add(gazebo_model_name)

        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {gazebo_model_name}: {e}")

    def load_scene(self, setup_name):
        """ Reads the CSV file and spawns models for a specific setup """
        rospy.init_node('spawn_scene', anonymous=True)
        self.delete_existing_models()  # Clear previous scene

        with open(CSV_FILE, "r") as file:
            reader = csv.DictReader(file)
            objects_poses = []
            for row in reader:
                if row["setup"] == setup_name:
                    model_name, id, x, y, z, roll, pitch, yaw = row["model_name"], int(row["id"]), float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                    if model_name != "label":
                        objects_poses.append([x, y])
                    print("model_name", model_name)
                    self.spawn_model(id, model_name, x, y, z)
                    rospy.sleep(0.1)
                    self.rotate_model(id, model_name, roll, pitch, yaw)

        return objects_poses

if __name__ == "__main__":
    rospy.init_node('spawn_scene', anonymous=True)

    spawn = SpawnerModels()
    camera = SpawnerCamera(init_point=spawn.init_pose)

    while not rospy.is_shutdown():
        scene_name = input("Enter setup name (or 'exit' to quit): ").strip()
        if scene_name.lower() == 'exit':
            spawn.delete_existing_models()
            rospy.loginfo("Exiting script...")
            break
        objects_poses = spawn.load_scene(scene_name)

        camera.goal_poses = objects_poses
        camera.move_camera()