
# Robotics Path Planning and Simulation

This repository contains Python modules for simulating robotic motion, map handling, and path planning using the D* Lite algorithm. Folder task 3 is most important. 

---

## Features

- **Simulator**: Real-time simulation of robot motion using Pygame.
- **Path Planning**: Implementation of the D* Lite algorithm for dynamic pathfinding.
- **Map Management**:
  - Subscribe to and publish map data using ROS2 topics.
  - Handle obstacles dynamically during the simulation.
- **Integration**: Combines ROS2 for messaging with Pygame for visualization.

---

## File Overview in task3 folder

### 1. `simulator.py`
- Simulates the robotic environment and movement.
- Features:
  - Dynamic obstacle handling.
  - Converts between real-world and pixel coordinates.
  - Publishes and subscribes to map updates.
  - Real-time visualization using Pygame.

### 2. `map_subscribe.py`
- Listens for map updates over ROS2 topics.
- Saves the received map data to a CSV file for use by other modules.
- Subscription Topic: `rew_map`.

### 3. `map_publish.py`
- Reads a map from a CSV file and publishes it as a ROS2 topic.
- Publication Topic: `pub_map`.

### 4. `DStarLite.py`
- Implements the D* Lite algorithm for path planning.
- Features:
  - Dynamic obstacle re-planning.
  - Calculates the shortest path between a start and goal node.
  - Handles changes in the environment during runtime.

---

## Prerequisites

- **Python 3.7+**
- **Dependencies**:
  - `pygame`
  - `numpy`
  - ROS2 and the `std_msgs` library.
- Ensure the required ROS2 environment is installed and configured.

---

## Installation

1. Clone the repository:
   ```bash
   git clone <this repository>
   cd <this repository>
   ```

2. Install dependencies:
   ```bash
   pip install pygame numpy
   ```

3. Ensure ROS2 is running, and set up the environment variables required for ROS2 communication.

---

## Usage

1. **Launch the Simulator**:
   ```bash
   python simulator.py
   ```
   - Visualizes the robot's path and handles real-time updates.

2. **Publish a Map**:
   ```bash
   python map_publish.py
   ```
   - Reads a map from `maps/map.csv` and publishes it to the topic `pub_map`.

3. **Subscribe to Map Updates**:
   ```bash
   python map_subscribe.py
   ```
   - Subscribes to `rew_map` and saves updates to `maps/map.csv`.

4. **Run Path Planning**:
   - The simulator will handle dynamic path planning based on map data.

---

## License

This project is licensed under the [MIT License](LICENSE).
