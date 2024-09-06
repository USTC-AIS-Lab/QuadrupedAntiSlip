## QuadrupedAntiSlip
**QuadrupedAntiSlip** provides a proprioceptive slip detector and a slip recovery controller for reliable quadruped locomotion on slippery surfaces, which is integrated into an MPC locomotion strategy.

### Example
Slip detection on slippery surfaces(ice). Yellow and red dots represent slipping and stationary contacts, respectively.
![图片描述](/img/1.gif)

Locomotion on slippery surfaces(ice) with(left) and without(right) slip-recovery controller.
![图片描述](/img/2.gif)

The video is available at [here](https://www.youtube.com/watch?v=F0YUZEiboGE) on YouTube.

### Dependencies
The repo has the following dependencies. Make sure you installed them first:
+ [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 
+ [OSQP](https://osqp.org/) 
+ [OSQP](https://github.com/robotology/osqp-eigen) 

### Installation
#### Gazebo Simulation
The Gazebo Simulation is built as a docker image. You can create the container as follows:
Run
```
docker pull chalkchalk/a1_gazebo
```
to get the image.

Run
```
docker run -dit\
    --name a1_gazebo \
    --cap-add=SYS_PTRACE \
    --gpus all \
    --shm-size=4G \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=unix$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
    -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics \
    --privileged \
    --network=host\
    chalkchalk/a1_gazebo
```
to create the container.

Run
```
docker exec -it a1_gazebo bash
```
to open the terminal inside container.

Then, run
```
gazebo_ice
```
which is actually an alias to run the simulation with the ice map.

#### QuadrupedAntiSlip Controller

##### Clone the repository
Clone the repo into a ROS workspace
```
git clone git@github.com:USTC-AIS-Lab/QuadrupedAntiSlip.git
```

##### Build the project
Enter the workspace and run
```
catkin build
```

##### Run the project
Launch the roslaunch file to start the controller for simulation:
```
roslaunch doggy_control gazebodog.launch
```

Then, you can run the joy_node to control the quadruped with a joystick.
```
rosrun joy joy_node
```

### Acknowledgements
The MPC controller used in this repo is based on [A1-QP-MPC-Controller
](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller) provided by [ShuoYangRobotics](https://github.com/ShuoYangRobotics). 