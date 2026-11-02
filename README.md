# Nautilus Software

This repository contains the software required to operate the **PIGEON** the Autonomous Underwater Vehicle (AUV) from Nautilus Robotique, both in Gazebo simulation and on the real submarine**.

The project is built around **ROS 2 Humble**, ArduPilot, Gazebo and Python. It includes the simulation environment, sensor interfaces, computer vision pipelines and mission-related software used by Nautilus.

---

# 1. Repository Setup

## Navigate to the ROS 2 Workspace

The ROS 2 workspace is located inside `NautilusSW/nautilus_ws`.

```bash
cd ~/NautilusSW/nautilus_ws
```

## Git Workflow

After cloning the repository, you should normally be on the `dev` branch.

The `dev` branch is protected and should not be used directly for development. New work should be done on the appropriate `NSFW-XX` branch.

Check your current branch and repository status:

```bash
git status
```

Make sure your local repository is up to date:

```bash
git pull
```

Switch to the branch associated with the task you are working on:

```bash
git checkout NSFW-XX
```

Once you are on the correct branch, you can start making changes.

To stage all modified files:

```bash
git add .
```

Check which files have been modified or staged:

```bash
git status
```

Commit your changes:

```bash
git commit -m "NSFW-XX Description of changes"
```

For example:

```bash
git commit -m "NSFW-42 Add slalom detection to vision pipeline"
```

---

# 2. Building the ROS 2 Workspace

After cloning the repository for the first time, build the workspace:

```bash
cd ~/NautilusSW/nautilus_ws
colcon build
```

Then source the Nautilus workspace:

```bash
source install/setup.bash
```

For the simulation, the ArduPilot ROS 2 workspace must also be sourced:

```bash
source ~/ardu_ws/install/setup.bash
```

> **Important:** Every new terminal used to run a Nautilus ROS 2 node must source the Nautilus workspace:
>
> ```bash
> source ~/NautilusSW/nautilus_ws/install/setup.bash
> ```

The `ardu_ws` workspace only needs to be sourced in terminals that require the ArduPilot simulation packages.

---

# 3. Simulation

## 3.1 Gazebo Model Path

Gazebo must know where to find both the ArduPilot and Nautilus simulation models.

This configuration only needs to be done once.

Open your `.bashrc`:

```bash
nano ~/.bashrc
```

Go to the bottom of the file and add:

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$HOME/NautilusSW/nautilus_ws/src/nautilus_bringup/models:${GZ_SIM_RESOURCE_PATH}
```

Save and exit Nano:

```text
Ctrl + O
Enter
Ctrl + X
```

Then either open a new terminal or reload the `.bashrc`:

```bash
source ~/.bashrc
```

---

## 3.2 Launch the Simulation

Open a terminal and run:

```bash
cd ~/NautilusSW/nautilus_ws
source install/setup.bash
source ~/ardu_ws/install/setup.bash
ros2 launch nautilus_bringup orca_comp.launch.py
```

This launches the **Gazebo RoboSub competition environment** and spawns the Nautilus AUV.

---

## 3.3 Run the Vision Pipeline in Simulation

Open a new terminal:

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
```

Then start the YOLO vision pipeline:

```bash
ros2 run nautilus_sensors yolo_pipeline --sim --bbox
```

The pipeline processes the camera images, performs YOLO object detection and publishes the resulting data as ROS 2 topics.

---

## 3.4 Visualize the Camera Output

To visualize ROS 2 image topics, open another terminal:

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select the desired image topic from the `rqt_image_view` interface.

---

# 4. Real Submarine

The real Nautilus AUV uses the same ROS 2 workspace, but communicates with the physical sensors instead of the simulated Gazebo sensors.

Start by opening a terminal:

```bash
cd ~/NautilusSW/nautilus_ws
source install/setup.bash
```

---

## 4.1 DVL

The real DVL node can be started using:

```bash
ros2 run nautilus_sensors dvl_sensor_node
```

A fake DVL is also available for development and testing:

```bash
ros2 run nautilus_sensors fake_dvl
```

---

## 4.2 Camera Stream

The threaded camera stream can be started using:

```bash
ros2 run nautilus_sensors stream_threaded
```

This node is used to interface with the Nautilus camera system and publish the camera data through ROS 2.

---

## 4.3 Vision Pipeline

Start the Nautilus YOLO pipeline using:

```bash
ros2 run nautilus_sensors yolo_pipeline --real --bbox
```

The pipeline combines the camera input with the YOLO detection model to process objects detected by the submarine.

---

## 4.4 Vision Debugging Tools

Several additional ROS 2 executables are available for debugging the vision system.

### Edge Detector Tuning

```bash
ros2 run nautilus_sensors slider_edge_detector
```

---

# 5. Vision Dependencies

The vision pipeline uses **Ultralytics YOLO**.

Because newer NumPy versions can cause compatibility problems with the ROS 2 Humble version of `cv_bridge`, install Ultralytics while keeping NumPy below version 2:

```bash
pip install ultralytics "numpy<2"
```

You can verify the installed NumPy version with:

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

---

# 6. Useful ROS 2 Commands

## List Available Nautilus Sensor Executables

If you are unsure which nodes are available:

```bash
ros2 pkg executables nautilus_sensors
```

The package currently provides executables including:

```text
nautilus_sensors dvl_sensor_node
nautilus_sensors fake_dvl
nautilus_sensors slider_edge_detector
nautilus_sensors stream_threaded
nautilus_sensors yolo_detections_viewer
nautilus_sensors yolo_pipeline
nautilus_sensors yolo_profiling_viewer
```

## List Active Topics

```bash
ros2 topic list
```

## Display Topic Data

```bash
ros2 topic echo /topic_name
```

## Check Topic Frequency

This is particularly useful to verify that a sensor or camera is actively publishing:

```bash
ros2 topic hz /topic_name
```

## List Active Nodes

```bash
ros2 node list
```

## Get Information About a Node

```bash
ros2 node info /node_name
```

## Visualize Image Topics

```bash
ros2 run rqt_image_view rqt_image_view
```

---

# 7. Adding a New Python ROS 2 Node

Python files should normally be launched through ROS 2 rather than directly using:

```text
python3 path/to/file.py
```

The `nautilus_sensors` package uses its `CMakeLists.txt` to register Python scripts as ROS 2 executables.

After adding a new executable to the package, rebuild it:

```bash
cd ~/NautilusSW/nautilus_ws
colcon build --packages-select nautilus_sensors
source install/setup.bash
```

Then verify that ROS 2 can find the executable:

```bash
ros2 pkg executables nautilus_sensors
```

It can then be started using:

```bash
ros2 run nautilus_sensors executable_name
```

---

# 8. Quick Start

## Simulation

### Terminal 1 — Gazebo + Nautilus

```bash
cd ~/NautilusSW/nautilus_ws
source install/setup.bash
source ~/ardu_ws/install/setup.bash
ros2 launch nautilus_bringup orca_comp.launch.py
```

### Terminal 2 — Vision Pipeline

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run nautilus_sensors yolo_pipeline
```

### Terminal 3 — Image Visualization

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

---

## Real Submarine

### Terminal 1 — Camera

```bash
cd ~/NautilusSW/nautilus_ws
source install/setup.bash
ros2 run nautilus_sensors stream_threaded
```

### Terminal 2 — DVL

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run nautilus_sensors dvl_sensor_node
```

### Terminal 3 — Vision Pipeline

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run nautilus_sensors yolo_pipeline
```

### Terminal 4 — Image Visualization

```bash
source ~/NautilusSW/nautilus_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

# 9. Project Structure

The main ROS 2 workspace is located in:

```text
NautilusSW/
└── nautilus_ws/
    └── src/
```

The main Nautilus packages include:

```text
nautilus_bringup
    Simulation, Gazebo worlds, models and launch files

nautilus_sensors
    DVL, cameras, computer vision and sensor processing

nautilus_mission
    Autonomous mission logic
```

---

# Nautilus AUV

**PIGEON — Université de Sherbrooke**

PIGEON is an autonomous underwater vehicle developed by a team of engineering students from the **Université de Sherbrooke**.

The vehicle was developed to compete at **RoboSub**, where autonomous underwater vehicles must complete a series of underwater navigation, perception and manipulation tasks.

The Nautilus software architecture combines **ROS 2**, **ArduPilot**, **Gazebo**, computer vision and autonomous navigation to allow the same software architecture to be developed and tested in simulation before being deployed on the real submarine.
