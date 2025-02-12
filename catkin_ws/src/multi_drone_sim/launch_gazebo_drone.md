### Preporation: 

Before launching the world, set the folder path for world and models.
In multi_drone_sim/launch/launch_world.launch line 18 set path to wolrd in this package. 


Reelseance model and plugins
https://github.com/intel/gazebo-realsense/tree/master 


Also load model files to 
*/usr/share/gazebo-11/models*

**RealSeancePlugin.hh** to
*/usr/include/gazebo-11/gazebo*

Load this package to your workspace. 

### Launch simulation
In first terminal launch gazebo world: 

```sh
roslaunch multi_drone_sim launch_world.launch 
```

In second terminal window, enter the ArduCopter directory and start the SITL simulation 0:
```sh
cd ~/ardupilot/Tools/autotest 
./sim_vehicle.py -v ArduCopter -f gazebo-iris  --console -I0
```

Then create launch ros: 

```sh
roslaunch multi_drone_sim apm.launch
```

And open ground control station 

```sh
rosrun multi_drone_sim ground_station.py
```


### Useful Commands

- Takeoff

To initiate the takeoff sequence, follow these steps:

    Type 13 and press Enter.
    Then type 3 and press Enter to set the altitude (in meters).

- Example of Velocity Publisher

The velocity publisher is demonstrated in the file **mission_drone.py** at **line 170**. This function is essential for controlling the drone's velocity [v_x, v_y, v_z], yaw_rotation.

Testing from Ground Station

To test the velocity publisher from the ground_station.py, you can send the command "1". This will call the velocity publisher function and allow you to observe its behavior in a controlled test environment.

### Add gates

You can add or change gate or position in 

multi_drone_sim/worlds/multitul.world

use current gates as an example. 