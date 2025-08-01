# 🦾 J-PARSE: Jacobian-based Projection Algorithm for Resolving Singularities Effectively in Inverse Kinematic Control of Serial Manipulators


### [Shivani Guptasarma](https://www.linkedin.com/in/shivani-guptasarma/), [Matthew Strong](https://peasant98.github.io/), [Honghao Zhen](https://www.linkedin.com/in/honghao-zhen/), and [Monroe Kennedy III](https://monroekennedy3.com/)


_In Submission_

<img
  src="images/splash.png"
  alt="JPARSE splash"
  style="width:100%;"
/>

<!-- ![JPARSE Concept diagram](images/jparse_concept_fig.png)
 -->

[![Project](https://img.shields.io/badge/Project_Page-J_PARSE-blue)](https://jparse-manip.github.io)
[![ArXiv](https://img.shields.io/badge/Arxiv-J_PARSE-red)](https://arxiv.org/abs/2505.00306) 


## FOR ARGALLAB
Instructions for things to launch depending on what you want to do.

### JOYSTICK or SSIP/PUFF Teleop
```
roslaunch xarm7_gripper_moveit_config realMove_exec.launch robot_ip:=<add the robot ip> add_gripper:=true

roslaunch manipulator_control xarm_main_vel.launch use_space_mouse:=true use_space_mouse_jparse:=true

roslaunch xarm_teleop xarm_teleop.launch JOY:=true (or SNP:=true)
```

*NOTE:* The GUI needs to be edited that is the paradigm is 3 (meaning 3-axis joystick mapping), then no GUI is required. (7/31/25)

### Keyboard Teleop
Instructions to teleop robot with jparse and keyboard:

Launch the following (velocity_control param for the xarm7_gripper_moveit_config launchfile is left as false in this case):
```
roslaunch xarm7_gripper_moveit_config realMove_exec.launch robot_ip:=<add the robot ip> add_gripper:=true

roslaunch manipulator_control xarm_main_vel.launch use_space_mouse:=true use_space_mouse_jparse:=true

rosrun teleop_twist_keyboard teleop_twist_keyboard.py _stamped:=True _frame_id:=link_eef cmd_vel:=robot_action
```

Edits need to be still commpleted to test the robot after writing script for joystick and sip/puff teleop.

### Gripper Notes
In order for the gripper to work, you need to first command the gripper in Rviz, then the the gripper will work. I am working on another fix for this. I am thinking it has to do with enabling the gripper potentially (8/1/25).
This has been fixed by enabling the gripper (8/1/25).

## FOR ARGALLAB TO DOS (not in order of to do):
- [x] Need to implement gripper in the `xarm_vel_experimenter.py` -- this is a function call to the api!!! the topic is `/gripper_action`
- [ ] self-avoidance collision -- wait till ros2 port
- [ ] work-around for the joint limits so that the robot does *not* immediately need to be power cycled -- wait till ros2 port
- [ ] **port to ros2 (need pinnochio) --> working on this!**

## Dependencies needed
There are all GitHub repos that can be cloned and added to this workspace.
- `catkin_simple`
- `teleop_twist_keyboard`
- `xArm-Python-SDK`
- `xarm_ros`

## Quick Start with Docker

To build the Docker image for the our environment, we use VNC docker, which allows for a graphical user interface displayable in the browser.

### Use the Public Docker Image (Recommended)

We have created a public Docker image that you can pull! 
Steps:

```sh
docker pull peasant98/jparse
docker run --privileged -p 6080:80 --shm-size=512m -v <path to jparse repo>:/home/ubuntu/Desktop/jparse_ws/src peasant98/jparse
```

### Build the Image Yourself

You can build the docker image yourself! To do so, follow the below steps:
```sh
cd Docker
docker build -t jparse .
docker run --privileged -p 6080:80 --shm-size=512m -v <path to jparse repo>:/home/ubuntu/Desktop/jparse_ws/src jparse

```

### Instructions for Argallab
We will need to build the docker image ourself (but I guess sincewe now have it done once, need to remind myself to upload to argallab docker hub).
Below are the instructions for building the image ourself and the docker container:
```sh
cd <naviagte to your git clone of this>/Docker
docker build -t jparse .

sudo docker run -it --privileged \
-v /dev:/dev \
-v /home/demiana/workspaces/jparse_ws/src:/home/jparse_ws/src \
-e DISPLAY \
-e QT_X11_NO_MITSHM=1 \
--name argallab_jparse \
--net=host \
jparse:latest
```

This will start create the container as well. For future, to start the container:
```
sudo docker start -i argallab_jparse
```

If display is having issues, then in the local machine (outside docker) enter the following command:
```
xhost +local:docker 
```

### Note

We are working on a Python package that you can easily import into your project. We envision the below:

```py
import jparse
```

Stay tuned!

### Dependencies
*Note: these are handled in the Docker image directly, and are already installed!*

1. [Catkin Simple](https://github.com/catkin/catkin_simple): https://github.com/catkin/catkin_simple
2. [HRL KDL](https://github.com/armlabstanford/hrl-kdl): https://github.com/armlabstanford/hrl-kdl 


## Running Velocity Control (XArm) Example

### Simulation
To run the XArm in simulation, first run
```bash
roslaunch manipulator_control xarm_launch.launch
```

#### Run Desired Trajectory
Next, run one of the trajectory generation scripts. This can either be the ellipse that has poses within and on the boundary of the reachable space of the arm (to test stability):
```bash
roslaunch manipulator_control full_pose_trajectory.launch robot:=xarm
```
or for passing through the type-2 singularity (passing directly above the base link): 
```bash
roslaunch manipulator_control se3_type_2_singular_traj.launch robot:=xarm
```
To have more control over keypoints (stop at major and minor axis of ellipse), run
```bash
roslaunch manipulator_control se3_type_2_singular_traj.launch robot:=xarm key_points_only_bool:=true frequency:=0.1 use_rotation:=false
```
or 
```bash
roslaunch manipulator_control full_pose_trajectory.launch robot:=xarm key_points_only_bool:=true frequency:=0.1 use_rotation:=false
```
(here frequency specifies how much time is spent at each keypoint).
or 
```bash
roslaunch manipulator_control line_extended_singular_traj.launch robot:=xarm key_points_only_bool:=true frequency:=0.2 use_rotation:=false
```
(line trajectory that goes from over the robot, to out of reach in front of the robot.)

#### Run Control Method
```bash
roslaunch manipulator_control xarm_main_vel.launch is_sim:=true show_jparse_ellipses:=true phi_gain_position:=2.0 phi_gain_angular:=2.0  jparse_gamma:=0.2 method:=JParse 
```

The arguments are 
| Parameter   | Attribute Description |
|------------|----------------------|
| `is_sim`   | Boolean for sim or real |
| `show_jparse_ellipses`   | Boolean for showing position JParse ellipsoids (for that method only) in rviz |
| `phi_gain_position`   | Kp gain for JParse singular direction position |
| `phi_gain_angular`   | Kp gain for JParse singular direction orientation |
| `jparse_gamma`   | JParse threshold value gamma |
| `method`   |  "JParse", "JacobianPseudoInverse" (basic); "JacobianDampedLeastSquares"; "JacobianProjection"; "JacobianDynamicallyConsistentInverse" |


## Real Robot Velocity Control 
### XArm Velocity Control Example
To run on the physical Xarm, the update is to use
```bash
roslaunch manipulator_control xarm_main_vel.launch is_sim:=false method:=JParse 
```
Recommended methods for physical system (to avoid unsafe motion) is: "JParse", "JacobianDampedLeastSquares"

<!-- For kinova arm! -->
<!-- ### Kinova Gen 3 Velocity Control Example
Run the Kinova environment
```bash
roslaunch manipulator_control kinova_gen3.launch
```

#### Select Trajectory
Run desired Line Extended keypoints trajectory:
```bash
roslaunch manipulator_control line_extended_singular_traj.launch robot:=kinova key_points_only_bool:=true frequency:=0.1 use_rotation:=false
```

Run elliptical keypoints trajectory
```bash
roslaunch manipulator_control full_pose_trajectory.launch robot:=kinova key_points_only_bool:=true frequency:=0.06 use_rotation:=false
```

#### Select Control
Run the Method: 
```bash
roslaunch manipulator_control kinova_vel_control.launch is_sim:=true show_jparse_ellipses:=true phi_gain_position:=2.0 phi_gain_angular:=2.0  jparse_gamma:=0.2 method:=JParse 
``` -->

## Running JParse with the SpaceMouse controller

You can also run JParse with a human teleoperator using a SpaceMouse controller. This will allow for a fun sandbox to verify JParse. 

We plan to extend this to a simple learning policy as well. The code for that (collecting data, training a policy, and running inference) will be published soon!

To run, you can run

```sh
# run the real robot 
roslaunch manipulator_control xarm_real_launch.launch using_spacemouse:=true

# run the jparse method with or without jparse control
roslaunch xarm_main_vel.launch use_space_mouse:=true use_space_mouse_jparse:={true|false}

# run the spacemouse example!! Make sure the use_native_xarm_spacemouse argument is OPPOSITE of use_space_mouse_jparse.
roslaunch xarm_spacemouse_teleop.launch use_native_xarm_spacemouse:={true|false}
```
   
## Run C++ JParse publisher and service
This allows for publishing JParse components and visualizing using a C++ script
```bash
roslaunch manipulator_control jparse_cpp.launch jparse_gamma:=0.2  singular_direction_gain_position:=2.0 singular_direction_gain_angular:=2.0
```

The arguments are 
| Parameter   | Attribute Description |
|------------|----------------------|
| `namespace`   | namespace of the robot (e.g. xarm) |
| `base_link_name`   | Baselink frame |
| `end_link_name`   | end-effector frame |
| `jparse_gamma`   | JParse gamma value (0,1) |
| `singular_direction_gain_position`   | gains in singular direction for position |
| `singular_direction_gain_angular`   |  gains in singular direction for orientation |
| `run_as_service` | (boolean) true/false | 

For running as a service: 
```bash
roslaunch manipulator_control jparse_cpp.launch run_as_service:=true
```
Then to run service from a terminal (Xarm example): 

```bash
rosservice call /jparse_srv "gamma: 0.2
singular_direction_gain_position: 2.0
singular_direction_gain_angular: 2.0
base_link_name: 'link_base'
end_link_name: 'link_eef'" 
```
To see versatility, simply change the kinematic chain for the JParse solution for that segment. To view options for your kinematic tree:
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

To test with the robot (using a python node to control the arm, with JParse coming from C++), first run script above, then:
```bash
roslaunch manipulator_control xarm_python_using_cpp.launch is_sim:=true phi_gain_position:=2.0 phi_gain_angular:=2.0  jparse_gamma:=0.2 use_service_bool:=true 
```
This has same parameters as the python version, but with the service versus message option. Message is faster/cleaner, but service is very versatile: 
| Parameter   | Attribute Description |
|------------|----------------------|
| `use_service_bool`   | True: use service, False: use message|
| `jparse_gamma`   | JParse gain (0,1)|
| `phi_gain_position`   | gain on position component|
| `phi_gain_angular`   | gain on angular component|
| `is_sim`   | use of sim versus real (boolean)|
