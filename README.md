# Final Assignment

This is the private repository for Lab 4, **Group 88**.

- NetID: gfernandeznesp
- Student Number: 5228174
- Name: Gonçalo Fernandez Nespral Vaz
- Email: G.FernandezNespralVaz-1@student.tudelft.nl

Course: RO47003 - Robot Software Practicals 2025-2026, TU Delft

## Compilation Instructions

To correctly build this project, a workspace must first be created with the relevant source code:

```
mkdir -p ~/fa_ws/src && cd ~/fa_ws/src
```

Next, clone the two repositories containing the packages that will be used. `group88` is this repository, and `ro47003_mirte_simulator` cotains various package related to simulating mirte and a pedestrian image detection package.

```
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/lab4/group88.git
```

```
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/ro47003_mirte_simulator.git
```

Now, source the ROS underlay and install required dependencies.

```
source /opt/ros/humble/setup.bash
```

```
cd .. && rosdep install -i --from-path src --rosdistro humble -y
```

Finally, build the project

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo.
```

and source the overlay!

```
source install/setup.bash
```

Verify that the installation was successfull with

```
colcon list
```

This should result in the following output:

```bash
control_barrel_world	src/group88/control_barrel_world	(ros.ament_cmake)
detection_3d_to_markers	src/ro47003_mirte_simulator/detection_3d_to_markers	(ros.ament_cmake)
mirte_gazebo	src/ro47003_mirte_simulator/mirte_gazebo	(ros.ament_cmake)
mirte_msgs	src/ro47003_mirte_simulator/mirte_msgs	(ros.ament_cmake)
mirte_teleop	src/ro47003_mirte_simulator/mirte_teleop	(ros.ament_cmake)
pcl_obstacle_detector	src/group88/pcl_obstacle_detector	(ros.ament_cmake)

```

In addition, the directory structure after following these instructions should be the following:

```
.
└── src
    ├── group88
    │   ├── control_barrel_world
    │   ├── LABGROUP.md
    │   ├── media
    │   ├── pcl_obstacle_detector
    │   └── README.md
    └── ro47003_mirte_simulator
        ├── detection_3d_to_markers
        ├── mirte_gazebo
        ├── mirte_msgs
        ├── mirte_teleop
        └── README.md

```

This can be verified with

```
tree -L 3 -I build/ -I install -I log/
```

## Launching the Project

A launchfile is included to facilitate the startup process. Run it with the following command.

```
ros2 launch control_barrel_world solution.launch.xml
```

Note that this launch file includes important setup parameters for the mirte control node `control_barrel_world_node`. These can be edited directly from the launch file `solution.launch.xml`.

```xml
    <!-- Launch the control barrel world nodes -->
    <node pkg="control_barrel_world" exec="control_barrel_world" name="control_barrel_world_node" output="screen">
        <param name="publish_interval_ms" value="100"/>
        <param name="pedestrian_area_thresh" value="2500.0"/>  <!-- Must be a float! -->
        <param name="linear_speed" value="0.3"/>
        <param name="angular_speed" value="0.5"/>
    </node>

```

## Project Outline

As mentioned previously, the project consists of the following four packages from `ro47003_mirte_simulator`
```
detection_3d_to_markers	src/ro47003_mirte_simulator/detection_3d_to_markers
mirte_gazebo	        src/ro47003_mirte_simulator/mirte_gazebo
mirte_msgs	            src/ro47003_mirte_simulator/mirte_msgs
mirte_teleop	        src/ro47003_mirte_simulator/mirte_teleop
```
along with two additional packages contained in this repository.

```
control_barrel_world	src/group88/control_barrel_world
pcl_obstacle_detector	src/group88/pcl_obstacle_detector
```

### Directory Structure

More specifically, the `pcl_obstacle_detector` package is structured as:

```
src/group88/pcl_obstacle_detector
├── CMakeLists.txt
├── include
│   └── pcl_obstacle_detector
│       └── pcl_obstacle_detector.hpp
├── LICENSE
├── package.xml
└── src
    ├── callbacks.cpp
    ├── main.cpp
    ├── node.cpp
    └── proc.cpp

```

In addition, the `control_barrel_world` package is structured as:

```
src/group88/control_barrel_world
├── CMakeLists.txt
├── include
│   └── control_barrel_world
│       └── control_barrel_world.hpp
├── launch
│   └── solution.launch.xml
├── LICENSE
├── package.xml
└── src
    ├── callbacks.cpp
    ├── main.cpp
    └── node.cpp
```

### Topics and Nodes

When running the launch file, the following nodes are created:

```
/camera_controller_top_down
/detection_3d_to_markers_node
/detection_3d_to_markers_node
/gazebo
/mirte/camera_controller_depth
/mirte/differential_drive_controller
/mirte/gazebo_ros_p3d
/mirte/gripper_bumper
/mirte/gripper_bumper2
/mirte/joint_state_publisher
/mirte/robot_state_publisher
/mirte/spawn_urdf
/opencv_person_detector_node
/rviz2
/transform_listener_impl_5946910ff230
/transform_listener_impl_64c158173ca0

/control_barrel_world_node
/pcl_obstacle_detector_node
```

The final two nodes listed here are started by the `control_barrel_world` and `pcl_obstacle_detector` executables, located in their respective packages.

The interaction between these nodes can be visualized with `rqt_graph`.

![rqt_graph](media/rosgraph.png "Title")

As can be seen the `/control_barrel_world_node` subscribes to `/pedestrians` and `/detections`, adn then writes to `/mirte/cmd_vel`.

Furthermore the `/pcl_obstacle_detector_node` subscribes to `/mirte/camera_depths/points` and writes to `/detections`.



