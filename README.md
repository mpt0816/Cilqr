# Trajectory Planner ROS Package

C++/ROS Source Codes for Trajectory planning for autonomous driving using ilqr solver.

![OnRoadPlanning](resources/static.png)

## 1. Installation

Requirements

* ROS Melodic or later
* Python3

Clone repository to any catkin workspace and compile workspace

```shell
cd ~/catkin_ws/src
git clone https://github.com/mpt0816/Cilqr.git
cd ..
catkin_make
source devel/setup.bash
```

## 2. Example


https://user-images.githubusercontent.com/85840949/150943617-f949d10d-c1be-424f-9530-1a21a5c67eef.mp4


Example test case with 6 pedestrians, 3 moving vehicles and 2 static vehicles.

```shell
roslaunch planning pedestrian_test.launch
```

**Click anywhere in Rviz window with the `2D Nav Goal` Tool to start planning.**

Generate and run new random case:

```shell
roslaunch planning random_pedestrian_test.launch
```

## 4. Generate Safety Corridor

<video src="./resources/corridor_generator_1.webm"></video>


## 3. Acknowledgement

Special thanks to [Bai Li](https://github.com/libai1943/CartesianPlanner) for ros simulation environment

