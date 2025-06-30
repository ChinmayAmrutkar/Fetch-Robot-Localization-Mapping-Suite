# Fetch-Robot-Localization-Mapping-Suite
This package provides robust **2D SLAM (GMapping)** and **localization (AMCL)** workflows for the Fetch robot, designed for ROS Melodic. It supports both simulation and real-robot operation, enabling you to create maps and localize the robot using laser and odometry data.

---

## 📁 Directory Structure
```
fetch_localization/
├── build/ # Build artifacts (ignore) 
├── devel/ # Development environment (ignore)
└── src/
  ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
  └── fetch_amcl_project/
    ├── CMakeLists.txt
    ├── launch/
    │ ├── amcl.launch
    │ └── fetch_gmapping.launch
    ├── maps/
    │ ├── playground_map.pgm
    │ └── playground_map.yaml
    ├── package.xml
    ├── rviz/
    │ ├── amcl_config.rviz
    │ └── gmapping_config.rviz
    ├── scripts/
    │ └── log_poses.py
    └── src/
```

---

## 🧰 System Requirements

- **Ubuntu 18.04**
- **ROS Melodic**
- **Fetch simulation packages** (or real robot)
- **Laser scan & odometry topics** available

**Install required ROS packages:**
```bash
sudo apt install -y ros-melodic-slam-gmapping ros-melodic-map-server ros-melodic-amcl
```


---

## 🚦 Workflow 1: Mapping with GMapping (SLAM)

### 1. Start Simulation or Real Robot

- Launch your simulation (e.g., Gazebo) or bring up the real Fetch robot.
- Make sure `/scan` (laser) and `/odom` (odometry) topics are published.

### 2. Start GMapping
```bash
roslaunch fetch_amcl_project fetch_gmapping.launch
```
- This runs the `slam_gmapping` node, subscribing to `/scan` and `/odom`.
- Check your TF tree for correct transforms (`base_link` <-> `odom`).

### 3. Visualize Mapping in RViz

```bash
rviz -d $(rospack find fetch_amcl_project)/rviz/gmapping_config.rviz
```

- Add the `Map` display to see the occupancy grid being built.

### 4. Drive the Robot

- Use teleop or navigation commands to move the robot and explore the environment:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- The map will grow as the robot explores.

---

## 💾 Saving the Map

After mapping:

```bash
rosrun map_server map_saver -f $(rospack find fetch_amcl_project)/maps/playground_map
```
- This saves `playground_map.pgm` and `playground_map.yaml` in the `maps/` directory.

---

## 🚦 Workflow 2: Localization with AMCL

### 1. Launch AMCL

```bash
roslaunch fetch_amcl_project amcl.launch
```

- Loads the saved map and starts the `amcl` node for localization.
- Robot localizes itself using laser and odometry data.

### 2. Visualize in RViz

```bash
rviz -d $(rospack find fetch_amcl_project)/rviz/amcl_config.rviz
```
- View the robot's pose and particle cloud on the map.

---

## 🛠️ Troubleshooting

| Problem                        | Solution                                                        |
|--------------------------------|-----------------------------------------------------------------|
| Map not building               | Ensure `/scan` and `/odom` are publishing.                      |
| TF errors (missing transforms) | Check your TF tree; use `robot_state_publisher` if needed.      |
| Map saving fails               | Check directory permissions and topic names.                    |
| AMCL not localizing            | Ensure map is loaded and topics match launch config.            |

---

## 🗂️ Additional Utilities

- **scripts/log_poses.py**  
  Example Python script for logging robot poses during mapping or localization.

---

## 🔗 References

- [gmapping ROS Wiki](http://wiki.ros.org/gmapping)
- [AMCL ROS Wiki](http://wiki.ros.org/amcl)
- [Clearpath Robotics: ROS Navigation Basics](https://docs.clearpathrobotics.com/docs/ros1noetic/ros/ros/tutorials/ros101/intermediate/ros_navigation_basics/)

---

## 🏷️ License

MIT License

---

## 👤 Author

**Chinmay Amrutkar**  
M.S. Robotics and Autonomous Systems – AI, Arizona State University  
GitHub: [@ChinmayAmrutkar](https://github.com/ChinmayAmrutkar)


