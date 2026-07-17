# AMR Leader–Follower System with Real-Time Web Dashboard

A ROS 2-based autonomous multi-robot system demonstrating intelligent leader–follower navigation within a Gazebo industrial warehouse simulation. This project showcases a complete Autonomous Mobile Robot (AMR) software stack by integrating autonomous navigation, localization, sensor fusion, simulation, and a custom web-based monitoring dashboard.

The system leverages the ROS 2 Navigation Stack (Nav2), AMCL, Robot Localization (EKF), SLAM Toolbox, Gazebo Harmonic, and a custom C++ leader–follower controller to enable coordinated multi-robot navigation. A custom HTML/CSS/JavaScript dashboard communicates with ROS 2 through WebSockets to provide real-time visualization of robot telemetry, navigation status, maps, costmaps, and sensor data.

---

## Features

### Leader–Follower Navigation

- Custom C++ follower controller using TF transformations
- Dynamic leader tracking
- Safe following distance maintenance
- Multi-robot coordination

### Autonomous Navigation

- Nav2 Navigation Stack
- Global path planning
- Local path planning
- Waypoint navigation
- Recovery behaviors

### Obstacle Avoidance

- Dynamic obstacle detection
- Collision Monitor integration
- Local and global costmaps
- Automatic path replanning

### Localization and Mapping

- AMCL localization
- SLAM Toolbox support
- Occupancy grid mapping
- TF frame management

### Sensor Fusion

- IMU integration
- Wheel odometry
- Robot Localization (EKF)
- Accurate pose estimation

### Simulation

- Gazebo Harmonic simulation
- Industrial warehouse environment
- Multiple autonomous mobile robots
- ROS-Gazebo bridge integration

### Web Dashboard

- HTML, CSS, and JavaScript frontend
- roslibjs integration
- rosbridge WebSocket communication
- Live robot telemetry
- Velocity visualization
- Robot pose monitoring
- Costmap visualization
- IMU data
- LaserScan data
- Connection status monitoring

### Experiment Automation

- Automated navigation scripts
- Goal execution
- Performance analysis
- Simulation management

---

## Technology Stack

| Category | Technologies |
|----------|--------------|
| Framework | ROS 2 Jazzy / Humble |
| Programming Languages | C++, Python |
| Simulation | Gazebo Harmonic |
| Navigation | Nav2 |
| Localization | AMCL |
| Sensor Fusion | Robot Localization (EKF) |
| Mapping | SLAM Toolbox |
| Sensors | LiDAR, IMU, Wheel Odometry |
| Communication | rosbridge_server, tf2_web_republisher |
| Web Dashboard | HTML, CSS, JavaScript, roslibjs |
| Visualization | RViz2 |
| Build System | colcon |
| Middleware | DDS |
| Operating System | Ubuntu 24.04 |

---

## Installation

### Clone the Repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/<username>/amr_leader_follower.git
```

### Install Dependencies

```bash
sudo apt update

sudo apt install ros-${ROS_DISTRO}-navigation2
sudo apt install ros-${ROS_DISTRO}-nav2-bringup
sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-robot-localization
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
sudo apt install ros-${ROS_DISTRO}-tf2-web-republisher
sudo apt install ros-${ROS_DISTRO}-ros-gz
```

Install the remaining dependencies:

```bash
cd ~/ros2_ws

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

### Build the Workspace

```bash
cd ~/ros2_ws

colcon build --symlink-install
```

### Source the Workspace

```bash
source install/setup.zsh
```

or

```bash
source install/setup.bash
```

---

## Running the Project

### Launch the Complete System

```bash
ros2 launch amr_leader_follower leader_follower.launch.py
```

This launch file starts:

- Gazebo Harmonic
- Leader robot
- Follower robot
- Nav2 Navigation Stack
- AMCL
- Robot Localization (EKF)
- ros_gz_bridge
- Controllers
- Lifecycle managers

### Start the WebSocket Bridge

Open a new terminal.

```bash
source ~/ros2_ws/install/setup.zsh

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Start the TF republisher:

```bash
ros2 run tf2_web_republisher tf2_web_republisher
```

### Open the Dashboard

Navigate to the `dashboard/` directory and open:

```
amr_dashboard.html
```

The dashboard automatically connects to:

```
ws://localhost:9090
```

and displays:

- Robot position
- Robot orientation
- Velocity
- IMU data
- LaserScan
- Costmaps
- Robot status
- Live ROS topics
- TF information

### Send Navigation Goals

```bash
ros2 run amr_leader_follower goal_pose_straight.py
```

Additional automation scripts are available in the `scripts/` directory.

---

## Repository Structure

```text
amr_leader_follower/

├── config/
│   ├── nav2_params.yaml
│   ├── amcl_params_leader.yaml
│   ├── amcl_params_follower.yaml
│   ├── ekf_follower.yaml
│   └── controller_params.yaml
│
├── dashboard/
│   ├── amr_dashboard.html
│   ├── css/
│   ├── js/
│   └── assets/
│
├── include/
├── launch/
├── maps/
├── scripts/
│
├── src/
│   ├── follower_controller.cpp
│   ├── goal_pose_straight.py
│   └── ...
│
├── urdf/
├── worlds/
└── README.md
```

---

## System Architecture

```text
                    +----------------------+
                    |     Leader Robot     |
                    +----------+-----------+
                               |
                               | TF
                               |
                               v
                 +------------------------------+
                 |   follower_controller (C++)  |
                 +--------------+---------------+
                                |
                             cmd_vel
                                |
                                v
                    +----------------------+
                    |   Follower Robot     |
                    +----------+-----------+
                               |
          +--------------------+--------------------+
          |                                         |
     Wheel Odometry                            IMU Sensor
          |                                         |
          +----------------+------------------------+
                           |
                           v
             Robot Localization (EKF)
                           |
                           v
                        AMCL
                           |
                           v
                  Nav2 Navigation Stack
      +------------+------------+-------------+
      |            |            |             |
 Planner      Controller     Recovery    Collision
 Server         Server        Behaviors   Monitor
      |            |            |             |
      +------------+------------+-------------+
                           |
                    Local & Global Costmaps
                           |
                           v
                  Gazebo Harmonic Simulation
                           |
                      ros_gz_bridge
                           |
                  ROS Topics / TF Frames
                           |
              rosbridge_server + TF2 Web
                           |
                           v
                HTML / CSS / JavaScript Dashboard
```

---

## ROS 2 Components

### Navigation

- Nav2
- Planner Server
- Controller Server
- Behavior Server
- BT Navigator
- Waypoint Follower
- Collision Monitor
- Docking Server

### Localization

- AMCL
- Robot Localization (EKF)
- TF2
- Robot State Publisher

### Sensors

- LiDAR
- IMU
- Wheel Encoder Odometry

### Simulation

- Gazebo Harmonic
- ros_gz_bridge
- gazebo_ros2_control

### Controllers

- Diff Drive Controller
- Joint State Broadcaster
- Custom Leader–Follower Controller

### Web Technologies

- rosbridge_server
- tf2_web_republisher
- roslibjs
- HTML5
- CSS3
- JavaScript

---

## ROS Topics

The project publishes and subscribes to topics including:

- `/cmd_vel`
- `/odom`
- `/scan`
- `/imu`
- `/tf`
- `/tf_static`
- `/map`
- `/amcl_pose`
- `/goal_pose`
- `/plan`
- `/global_costmap`
- `/local_costmap`
- `/joint_states`
- `/clock`

---

## Dashboard Features

The web dashboard provides:

- Live robot telemetry
- Robot pose monitoring
- Velocity visualization
- IMU monitoring
- LaserScan visualization
- Costmap visualization
- ROS connection status
- TF monitoring
- Navigation status

---

## Applications

- Autonomous Mobile Robots (AMRs)
- Warehouse Automation
- Multi-Robot Coordination
- Leader–Follower Navigation
- Robotics Research
- ROS 2 Education
- Navigation Algorithm Development
- Human-Robot Interaction Research

---

## Author

**Amin Ahmed G**

Robotics & Automation Engineering Student

Interested in Autonomous Mobile Robots (AMRs), ROS 2, Navigation, Localization, Multi-Robot Systems, and Full-Stack Robotics Development.

---

## License

This project is intended for educational, research, and learning purposes.
