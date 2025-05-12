# ROS2 Turtlesim GUI

## Project Description
This project implements a Graphical User Interface (GUI) for the ROS2 Turtlesim simulation. The GUI allows users to interact with the Turtlesim environment by providing controls to spawn, kill, and move turtles, as well as visualize information about the ROS2 graph (nodes, topics, services, actions) and the real-time pose of selected turtles.

The GUI is developed using Python and the Tkinter library, integrated with the ROS2 client library for Python (rclpy). It demonstrates key ROS2 concepts including:

- Creating ROS2 nodes
- Publishing messages (Twist for movement)
- Subscribing to topics (Pose for real-time position)
- Calling services (Spawn, Kill, Reset)
- Introspection of the ROS2 graph (listing nodes, topics, services, actions)
- Concurrent execution of a ROS2 node and a GUI using threading for responsiveness.

## Requirements
- An Ubuntu 22.04 LTS system.
- ROS2 Humble Hawksbill installed and sourced. (Refer to the official ROS2 Humble installation guide if needed).
- Python 3.
- Tkinter library for Python (python3-tk).

## Installation
1. Set up a ROS2 workspace:
   ```bash
   mkdir -p ~/turtlesim_gui_ws/src
   cd ~/turtlesim_gui_ws
   colcon build
   ```

2. Clone or create your package: Place the turtlesim_gui package directory (containing your package.xml, setup.py, and the turtlesim_gui Python module) inside the `~/turtlesim_gui_ws/src` directory.

3. Install Tkinter:
   ```bash
   sudo apt-get install python3-tk
   ```

## Building the Package
Navigate to the root of your workspace and build the package:

```bash
cd ~/turtlesim_gui_ws
colcon build --packages-select turtlesim_gui
```

## Running the Application
1. Source your workspace:
   ```bash
   source ~/turtlesim_gui_ws/install/setup.bash
   ```

2. Launch the turtlesim node first:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

3. In a new terminal (after sourcing ROS2 and your workspace), run the GUI:
   ```bash
   ros2 run turtlesim_gui turtlesim_gui
   ```

The GUI will connect to the running turtlesim node, allowing you to control turtles, set parameters, and monitor the simulation through the interface.
