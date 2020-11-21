This repo contains the code for 2 examples of the ICRA 2021 submission.

[Accompanying video](https://youtu.be/Ju5hv2gIlxw)

A standalone version of the solver can be found by following the link [Solver only](https://github.com/ControlTrees/solver).
This version needs way less dependencies (e.g. no ros) and doesn't contain the examples of the paper.


# Dependencies

### ros
The planner is intergated as a ros node. It has been developped with ros kinetic but should be compatible with merely any recent ros distro.
[Ros installation](https://www.ros.org/install/)

### gazebo
Car dynamic is simulated with Gazebo.
[Gazebo installation](http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros)

### osqp
We use the QP solver OSQP as a baseline (see paper).
[OSQP installation](https://osqp.org/docs/installation/cc++)

### other dependencies

In addition, the following packages are necessary: gnuplot, libjsoncpp-dev, libx11-dev, liblapack-dev, libf2c2-dev, libeigen3-dev, libglew-dev, freeglut3-dev.
They can be installed by calling `sudo apt install PACKAGE_NAME`.

# Build
Clone the repository and its submodules into your catkin workspace:
```bash
git clone --recursive git@github.com:ControlTrees/icra2021.git
```
This will clone the code related to the examples and two submodules (https://github.com/ControlTrees/rai, and https://github.com/ControlTrees/tamp).

### install gazebo models
The car, obstacle and pedestrian gazebo models are part of the repository and must be copied to the gazebo folder containing all models.
This gazebo folder is typicall under `~/.gazebo/models`.
```bash
cd icra2021/lgp_car_models
cp -r * ~/.gazebo/models
```

### build RAI
build rai

### build ros nodes
catkin_make

# Execute tests
Open a terminal in the folder corresponding to the `control_tree_car` package in `build` folder of the catkin workspace.
This is typically `${CATKIN_WORKSPACE}/build/icra_2021/control_tree_car` where `CATKIN_WORKSPACE` is the root of the catkin workspace.

The tests can be launched using ctest:
```bash
ctest .
```

# Launch examples

### Pedestrian example
Open two terminals in the catkin workspace.

In the first terminal, type the following command, it will launch the planner and the visualization RViz.
```bash
roslaunch control_tree_car pedestrian.launch
```

In the second terminal, type the following command, it will launch the simulator.
```bash
gzserver src/icra_2021/lgp_car_gazebo_plugin/world/pedestrian_4.world
```

![Image](control_tree_car/data/doc/pedestrians.png)

### Obstacle avoidance example
Open two terminals in the catkin workspace.

In the first terminal, type the following command, it will launch the planner and the visualization RViz.
```bash
roslaunch control_tree_car obstacle_avoidance.launch
```

In the second terminal, type the following command, it will launch the simulator.
```bash
gzserver src/icra_2021/lgp_car_gazebo_plugin/world/obstacle_avoidance_2.world
```

![Image](control_tree_car/data/doc/obstacles.png)
