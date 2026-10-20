# TurtleBot 4 Simulation, Navigation and YOLO

ROS 2 Humble workspace for simulating a TurtleBot 4 in Gazebo Fortress, creating or loading an occupancy map, localizing with AMCL, navigating with Nav2, and detecting objects from the robot camera with YOLOv8.

## Repository contents

| Package | Purpose |
| --- | --- |
| `robot_description` | TurtleBot 4 / Create 3 Xacro model, meshes, sensor frames and simulation plugins. |
| `robot_sim` | Gazebo Fortress worlds, robot spawning and ROS–Gazebo topic bridges. The `custom` world includes COCO objects for YOLO demonstrations. |
| `robot_node` | Keyboard teleoperation and the `nav2_yolo_patrol` Python node. The patrol visits three configured map poses while YOLO processes camera images. |
| `robot_mapping` | SLAM Toolbox launch/configuration and map saving. |
| `robot_navigation` | Nav2 parameters, AMCL/navigation launch files, RViz configuration and example occupancy maps. |

The provided maps are `robot_navigation/maps/my_custom_map.yaml` with its image `my_custom_map.pgm`, and `my_map.yaml` with `my_map.pgm`. The YAML image paths are relative to the YAML files.

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- Gazebo Fortress and ROS–Gazebo integration
- Nav2, SLAM Toolbox, Xacro, RViz and TurtleBot description dependencies
- Python OpenCV, `cv_bridge`, and Ultralytics YOLO

Install the ROS dependencies:

```bash
sudo apt update
sudo apt install -y git python3-pip python3-venv python3-opencv \
  python3-colcon-common-extensions python3-yaml \
  ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge
```

## Clone and build

The repository root is intended to be the workspace `src` directory:

```bash
mkdir -p ~/workspaces/turtlebot4_ws/src
cd ~/workspaces/turtlebot4_ws/src
git clone https://github.com/sooheetumi-11/turtlebot4_simulation.git .

cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Create a virtual environment that can still import the ROS Python packages, then install Ultralytics. Activate this environment in the terminal used to run the YOLO node:

```bash
cd ~/workspaces/turtlebot4_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install 'numpy<2' ultralytics
```

For each terminal, source ROS and the workspace. Activate `.venv` in the terminal that runs the detector:

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=69
# In the terminal running YOLO:
source .venv/bin/activate
```

## Run the simulation, localization and navigation

Start a separate terminal for each long-running launch. Keep the simulation and localization alive while Nav2 and the patrol node are running.

**Terminal 1 — Gazebo and robot:**

```bash
ros2 launch robot_sim sim.launch.py world:=custom enable_camera:=true
```

The custom world references three Open Robotics Fuel models (standing person, stop sign and chair). Gazebo needs network access the first time it downloads these models; after download, Fuel caches them locally.

**Terminal 2 — map server, AMCL and RViz:**

```bash
ros2 launch robot_navigation localization.launch.py \
  map:=$PWD/src/robot_navigation/maps/my_custom_map.yaml
```

In RViz, select **2D Pose Estimate**, click the robot's approximate position on the map and drag the arrow to match its heading. Wait for AMCL's scan and particle cloud to settle. Keep this launch running.

**Terminal 3 — Nav2:**

```bash
ros2 launch robot_navigation navigation.launch.py
```

Wait for Nav2 lifecycle nodes to become active before starting the patrol. Localization and navigation are separate launches in this workspace: localization provides `/map` and `map → odom`; Nav2 provides planning and control.

## Run the three-goal YOLO patrol

**Terminal 4 — detector and patrol:**

```bash
cd ~/workspaces/turtlebot4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=69
ros2 launch robot_node nav2_yolo_patrol.launch.py
```

The launch defaults to camera topic `/camera/image`, which is the image topic bridged by this repository, and publishes annotated images on `/yolo/detections_image`. To use a different camera topic:

```bash
ros2 launch robot_node nav2_yolo_patrol.launch.py \
  camera_topic:=/oakd/rgb/preview/image_raw
```

The node loads `yolov8n.pt` (Ultralytics downloads it if it is not already present), displays annotated frames in an OpenCV window, and publishes the annotated stream for RViz. Press `q` in the image window or `Ctrl+C` in the terminal to stop the node.

### Patrol goals

Edit the `waypoints` list in `robot_node/robot_node/nav2_yolo_patrol.py` to set three `(x, y, yaw)` poses in the map frame. The current example corresponds to the object locations in `robot_sim/worlds/custom.sdf`:

```python
waypoints = [
    (0.00,  1.50,  1.57),
    (0.00, -1.20, -1.57),
    (2.20, -1.50,  0.00),
]
```

Use RViz's **Nav2 Goal** tool and inspect `/goal_pose` to record a pose including heading. `/clicked_point` provides a position only. Rebuild after editing the node if the workspace was built without `--symlink-install`.

## Mapping and teleoperation (optional)

To create a new map instead of using a supplied map, start SLAM while the simulation is running:

```bash
ros2 launch robot_mapping mapping.launch.py
```

In another terminal, drive slowly to scan the area:

```bash
ros2 run robot_node teleop_keyboard
```

Save the resulting map while SLAM is still running:

```bash
ros2 launch robot_mapping save_map.launch.py \
  map_name:=$PWD/src/robot_navigation/maps/my_custom_map
```

The map saver writes a YAML and PGM pair. Stop teleoperation and SLAM before starting AMCL with the new map.

## Key interfaces

- `/scan`, `/odom`, `/tf`, `/clock`: simulation data used by localization and navigation.
- `/map`, `/initialpose`, `/amcl_pose`: map and AMCL localization interfaces.
- `/navigate_to_pose`: Nav2 action used by the patrol node to visit each goal in sequence.
- `/cmd_vel`: Nav2 velocity output consumed by the Gazebo bridge.
- `/camera/image`: camera image input in this repository.
- `/yolo/detections_image`: annotated image output from the detector.

Do not run keyboard teleoperation while Nav2 is controlling the robot; both would send velocity commands to `/cmd_vel`.
