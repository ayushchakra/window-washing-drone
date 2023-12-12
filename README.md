# Commands to launch the gazebo world
```bash
colcon build --packages-select sjtu_drone_bringup sjtu_drone_description
ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
```

# Dependencies
ros-humble-gazebo-plugins
ros-humble-gazebo-ros
rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*
