

#### To enter the docker use from host pc:
``` bash
docker exec -it ardupilot bash
```

### Start simulation (ALL IN DOCKER): 

In first terminal launch: (first run will take a few seconds)

```sh
roslaunch drone_sim launch_world.launch 
```

In second terminal window, enter the ArduCopter directory and start the SITL simulation (It will do compilation in first launch):

```sh
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

Then in third terminal:

```sh
roslaunch drone_sim apm_50.launch
```

### Remove ros logs is case low memory

```bash
rm -rf ~/.ros
```

[Go back to docker preporation](0_docker.md)