<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR Leader-Follower System with Real-Time Web Dashboard</title>
</head>
<body>

    <h1>AMR Leader-Follower System with Real-Time Web Dashboard</h1>
    <p>A ROS 2-based multi-robot system demonstrating autonomous leader-follower navigation within a Gazebo simulated environment. This project integrates full-stack robotics development, featuring dynamic path planning, obstacle avoidance, and a custom web UI for real-time telemetry and mapping visualization.</p>

    <h2>🚀 Features</h2>
    <ul>
        <li><strong>Multi-Robot Coordination:</strong> Reliable leader-follower behavior utilizing a custom C++ <code>follower_controller</code>.</li>
        <li><strong>Navigation Stack Integration:</strong> Utilizes Nav2 for dynamic costmap generation, obstacle avoidance, and path planning using tuned parameters.</li>
        <li><strong>Physics Simulation:</strong> Fully simulated industrial/warehouse environment using Gazebo (<code>industrial_world.sdf</code>).</li>
        <li><strong>Real-Time Web Dashboard:</strong> A custom-built HTML/JS interface (<code>dashboard/amr_dashboard.html</code>) that communicates with ROS 2 via WebSockets to display live telemetry, costmaps, and velocity gauges.</li>
        <li><strong>Analysis & Automation:</strong> Included Python scripts for running follow experiments and auditing SLAM performance.</li>
    </ul>

    <h2>🛠️ Tech Stack & Prerequisites</h2>
    <ul>
        <li><strong>Operating System:</strong> Ubuntu 24.04 (Noble Numbat)</li>
        <li><strong>Framework:</strong> ROS 2 (Jazzy / Humble)</li>
        <li><strong>Simulation:</strong> Gazebo (using <code>ros_gz_bridge</code>)</li>
        <li><strong>Web Integration:</strong> <code>rosbridge_server</code>, <code>tf2_web_republisher</code></li>
        <li><strong>Shell:</strong> Setup scripts are optimized for <code>zsh</code></li>
    </ul>

    <h2>📦 Installation & Setup</h2>
    <ol>
        <li>
            <strong>Clone the repository into your ROS 2 workspace:</strong>
            <pre><code>mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone &lt;your-repository-url&gt; amr_leader_follower</code></pre>
        </li>
        <li>
            <strong>Install ROS 2 dependencies:</strong>
            <p>Ensure you have the necessary web, navigation, and Gazebo bridge packages installed:</p>
            <pre><code>sudo apt update
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
sudo apt install ros-${ROS_DISTRO}-tf2-web-republisher
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
sudo apt install ros-${ROS_DISTRO}-ros-gz

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y</code></pre>
        </li>
        <li>
            <strong>Build the workspace:</strong>
            <pre><code>cd ~/ros2_ws
colcon build --symlink-install</code></pre>
        </li>
        <li>
            <strong>Source the environment:</strong>
            <pre><code>source install/setup.zsh</code></pre>
        </li>
    </ol>

    <h2>💻 Usage</h2>
    
    <h3>1. Launch the System</h3>
    <p>Start the primary launch file that brings up the Gazebo <code>industrial_world</code>, spawns the leader and follower AMRs, initializes the Nav2 stack, and starts the <code>ros_gz_bridge</code>:</p>
    <pre><code>ros2 launch amr_leader_follower leader_follower.launch.py</code></pre>

    <h3>2. Start the Web Bridge Connection</h3>
    <p>In a new terminal window, initialize the WebSocket bridge and TF republisher to allow the web dashboard to communicate with your ROS nodes:</p>
    <pre><code>source ~/ros2_ws/install/setup.zsh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
ros2 run tf2_web_republisher tf2_web_republisher</code></pre>

    <h3>3. Open the Dashboard</h3>
    <p>Navigate to the <code>dashboard/</code> directory and open <code>amr_dashboard.html</code> in your web browser. The dashboard will automatically connect to the <code>ws://localhost:9090</code> stream.</p>

    <h3>4. Execute Goal Commands</h3>
    <p>Use the provided scripts to send automated goals or run experiments:</p>
    <pre><code>ros2 run amr_leader_follower goal_pose_straight.py</code></pre>

    <h2>📂 Repository Structure</h2>
    <pre><code>amr_leader_follower/
├── config/                  # Tuning parameters for Nav2, AMCL, EKF, and Controllers
├── dashboard/               # AMR_dashboard.html and related web assets
├── include/                 # C++ Headers
├── launch/                  # Launch files (leader_follower.launch.py, camera_bridge, etc.)
├── maps/                    # Pre-generated industrial_world map files
├── scripts/                 # Utility scripts for experiments, analysis, and sim management
├── src/                     # C++ nodes (follower_controller.cpp) & Python bridges
├── urdf/                    # Robot description (2_wheel.xacro)
└── worlds/                  # Gazebo environment SDF files (industrial_world, obstacles)</code></pre>

    <h2>🏗️ System Architecture</h2>
    <ul>
        <li><strong><code>follower_controller</code>:</strong> Custom C++ node (<code>src/follower_controller.cpp</code>) responsible for tracking the leader's TF frames and calculating necessary velocity commands.</li>
        <li><strong>Nav2 & Localization:</strong> Heavily customized tuning via <code>config/nav2_params.yaml</code>, utilizing AMCL (<code>amcl_params_leader.yaml</code> / <code>follower.yaml</code>) and Robot Localization (<code>ekf_follower.yaml</code>).</li>
        <li><strong>Web UI Bridge:</strong> Uses <code>roslibjs</code> to subscribe to ROS topics and push data to the frontend DOM elements asynchronously.</li>
    </ul>

</body>
</html>
