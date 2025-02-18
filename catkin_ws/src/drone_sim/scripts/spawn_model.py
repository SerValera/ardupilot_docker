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

import time

# Paths
SDF_PATH = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/"
CSV_FILE = "/home/sim/ardupilot_docker/catkin_ws/src/drone_sim/models/scene_setup.csv"

# Store the currently spawned models
spawned_models = set()

def delete_existing_models():
    """ Deletes all currently spawned models before switching scenes """
    rospy.wait_for_service("/gazebo/delete_model")
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    
    for model in list(spawned_models):
        try:
            delete_model(model)
            rospy.loginfo(f"Deleted {model}")
            spawned_models.remove(model)
            rospy.sleep(0.1)
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to delete {model}: {e}")

def rotate_model(model_name, roll, pitch, yaw):
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        # Get the current model state
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        current_state = get_state(model_name, "world")

        if current_state.success:
            # Convert roll, pitch, yaw to quaternion
            quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

            # Define new model state with the same position
            state = ModelState()
            state.model_name = model_name
            state.pose.position = current_state.pose.position  # Keep original position
            state.pose.orientation.x = quaternion[0]
            state.pose.orientation.y = quaternion[1]
            state.pose.orientation.z = quaternion[2]
            state.pose.orientation.w = quaternion[3]

            # Call the service to update the model state
            resp = set_state(state)
            return resp
        else:
            print(f"Failed to get state for model {model_name}")

    except rospy.ServiceException as e:
        print("Service call failed:", e)

def spawn_model(model_name, x, y, z):
    """ Spawns a model in Gazebo at a given position """
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

    try:
        spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        spawn_service(model_name, model_xml, "", pose, "world")
        rospy.loginfo(f"Spawned {model_name} at ({x}, {y}, {z})")
        spawned_models.add(model_name)

    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn {model_name}: {e}")

def load_scene(setup_name):
    """ Reads the CSV file and spawns models for a specific setup """
    rospy.init_node('spawn_scene', anonymous=True)
    delete_existing_models()  # Clear previous scene

    with open(CSV_FILE, "r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row["setup"] == setup_name:
                model_name = row["model_name"]
                x, y, z, roll, pitch, yaw = float(row["x"]), float(row["y"]), float(row["z"]), float(row["roll"]), float(row["pitch"]), float(row["yaw"])
                spawn_model(model_name, x, y, z)
                rospy.sleep(0.1)
                rotate_model(model_name, roll, pitch, yaw)

if __name__ == "__main__":
    rospy.init_node('spawn_scene', anonymous=True)

    while not rospy.is_shutdown():
        scene_name = input("Enter setup name (or 'exit' to quit): ").strip()
        if scene_name.lower() == 'exit':
            delete_existing_models()
            rospy.loginfo("Exiting script...")
            break
        load_scene(scene_name)