# Window Washing Drone
## Project Description
Our goal is to, in simulation, cover a window on the first story of a building with a soapy water mixture while using IMU and GPS to navigate to known positions in the scene. The full documentation for this project can be found [here](https://sites.google.com/view/window-washing-drone).

## Installation
Currently, this project is only compatible with ROS2 Humble and has not been tested with any other versions. Along with a full ROS2 install, the following dependencies will be required:

```bash
sudo apt-get install ros-humble-gazebo-plugins ros-humble-gazebo-ros
cd window_washing_drone && rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*
```

# Launching the Drone
For our simulation environment and drone module, we leverage the work of the Shanghai Jiao Tong University, which was primarily developed for the testing of UAV algorithms. The source repository can be found [here](https://github.com/NovoG93/sjtu_drone).

To launch the simulation environment, run the following commands:
```bash
colcon build --packages-select sjtu_drone_bringup sjtu_drone_description
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

Then, to run our custom drone controls stack, run the following commands:
```bash
colcon build --packages-select drone_controls
ros2 run drone_controls state_estimation
ros2 run drone_controls command_drone
```

If you wish to change the set windows that you would like the drone to navigate over, simply update the parameter list (DESIRED_MAP_FEATURES) in `src/drone_controls/drone_controls/fly.py`. NOTE: The corner nodes are essentially to preventing the drone from colliding with the walls.

Additionally, we also support teleoperating the drone via the following command:
```bash
ros2 run drone_controls teleop
```