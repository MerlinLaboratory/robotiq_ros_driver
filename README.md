# robotiq_ros_driver

ROS metapackage based on the package developed by the [Control Robotics Intelligence Group](http://www.ntu.edu.sg/home/cuong/) from the [Nanyang Technological University, Singapore](http://www.ntu.edu.sg) and now customised for [Merlin Lab](http://merlin.deib.polimi.it/).

## Setup

  * Robotiq Hand-e (Universal Robots e-series)

## Installation

Go to your ROS working directory. e.g.
```{bash}
cd ~/catkin_ws/src
```

Clone these repository:
```{bash}
git clone https://github.com/MerlinLaboratory/robotiq_ros_driver.git
```

Install any missing dependencies using rosdep:
```{bash}
rosdep update
rosdep install --from-paths . --ignore-src -y
```

Now compile your ROS workspace. e.g.
```{bash}
cd ~/catkin_ws && catkin build
```

### Testing

The repo works with the real gripper only in case the gripper is attacched to the UR5e and the robotiq URcap is installed in the teach pendant. Conversely, the repo can only be used to simulated the gripper in Gazebo.

#### Testing in simulation

TODO

#### Testing with robot
1) Be sure to always source the appropriate ROS setup file, e.g:
```{bash}
source ~/catkin_ws/devel/setup.bash
```
You might want to add that line to your `~/.bashrc`

2) Physically connect to the ur5e through the ethernet cable and setup a static ip under the robot subnetwork (as of today: 192.168.125.X)

3) Try the launching the exaple script `urcap_cmodel_test`:
```{bash}
roslaunch robotiq_control urcap_cmodel_test.launch address:=ur_robot_ip
```
Expected output:
```
Simple C-Model Controller
-----
Current command:  rACT = 0, rGTO = 0, rATR = 0, rPR = 0, rSP = 0, rFR = 0
-----
Available commands

r: Reset
a: Activate
c: Close
o: Open
(0-255): Go to that position
f: Faster
l: Slower
i: Increase force
d: Decrease force
-->
```
