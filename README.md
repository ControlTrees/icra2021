# icra_2021

This repo contains the code for 2 examples of the ICRA 2021 submission.

[Accompanying video](https://youtu.be/Ju5hv2gIlxw)

## Installation

ros

gazebo

install models

clone, with submodules

build rai

catkin_make

## Execute tests

## Launch examples

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
