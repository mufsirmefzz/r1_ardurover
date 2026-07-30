# r1_ardurover
# GPS-Based Waypoint Navigation Using ArduRover with Obstacle Avoidance via the Nav2 Stack

## Installation Required and Prerequisites

### System Update & Core Build Tools
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git cmake build-essential g++ gcc curl gnupg2 lsb-release python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool libxml2-dev libxslt-dev
```

### Install ROS 2 Humble Desktop

**1. Set locale**
```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**2. Add ROS 2 GPG Key & Repository**
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**3. Install ROS 2 Humble Desktop**
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

**4. Source ROS 2 environment in bashrc**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**5. Initialize rosdep**
```bash
sudo rosdep init
rosdep update
```

### Install Required ROS 2 Navigation & Sensor Packages
```bash
sudo apt update && sudo apt install -y ros-humble-nav2-bringup ros-humble-nav2-costmap-2d ros-humble-nav2-planner ros-humble-nav2-controller ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager ros-humble-nav2-simple-commander ros-humble-robot-localization ros-humble-twist-mux ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-rplidar-ros ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-std-srvs
```

### Install Required Python Packages
```bash
pip3 install --upgrade pip
pip3 install pymavlink mavproxy pyserial future numpy transforms3d pyyaml
```

### Setup & Build ArduPilot SITL Firmware

**1. Clone ArduPilot repo in home directory**
```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot
```

**2. Run ArduPilot prerequisites installer script**
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
```

**3. Configure and compile ArduRover SITL binary**
```bash
./waf configure --board=sitl --frame=rover
./waf rover
```

**4. Verify that binary exists**
```bash
ls -l ~/ardupilot/build/sitl/bin/ardurover
```

### Configure Serial Port Permissions
```bash
sudo usermod -a -G dialout $USER

# Connection of the Pixhawk
sudo chmod 666 /dev/ttyACM2 2>/dev/null || true  

# Connection of the Lidar
sudo chmod 666 /dev/ttyUSB0 2>/dev/null || true  
```

### Build Ardurover Workspace
```bash
cd ~/ardurover_ws

# Clean previous build artifacts (if any)
rm -rf build/ install/ log/

# Install missing rosdep dependencies (optional check)
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace setup script
source install/setup.bash
```

## Mission Execution WorkFlow

### Terminal 1: Publish GPS Data and Heading of the Rover
**Description:** Connects to the Pixhawk FCU on `/dev/ttyACM2` via pymavlink, receives raw GPS fix and compass heading telemetry, and publishes them to ROS 2 topics (`/gps/fix` and `/gps/heading`).
```bash
cd ardurover_ws
python3 gps_with_heading.py
```
*Verification:* Check the `ros2 topic list` output to verify that the `/gps/fix` and `/gps/heading` topics are present.

### Terminal 2: Make the Ardurover Firmware Active (SITL Execution)
**Description:** Executes the compiled ArduRover SITL firmware binary configured for JSON model interface (TCP:5760) and streams MAVLink telemetry via UDP client to the ground station.
```bash
cd ~/ardupilot
./build/sitl/bin/ardurover --model JSON:127.0.0.1 -I0 --serial1=udpclient:<SYSTEM_IP>:14550
```
*Note:* Replace `<SYSTEM_IP>` with your host / GCS IP address (e.g. `10.227.142.69` or `192.168.1.249`).

### Terminal 3: Run the Lidar Active
**Description:** Launches the official ROS 2 driver for RPLiDAR S2 scanner to record surrounding obstacles and publish 2D laser scan data.
```bash
ros2 launch rplidar_ros rplidar_s2_launch.py
```
*Verification:* Check the `ros2 topic list` output to verify that the `/scan` topic is present.

### Terminal 4: Hardware Loop Connection with Ardurover
**Description:** Builds the workspace and runs the ROS 2 HITL bridge node to inject ROS 2 sensor telemetry (`/raw IMU`, `/gps/fix`, `/gps/heading`, `/r1a004/wheel_odom`) into ArduRover over MAVLink TCP, while translating ArduRover servo outputs into `/cmd_vel_ardupilot`.
```bash
cd ardurover_ws
colcon build --symlink-install
source install/setup.bash  
ros2 run ardurover_hitl hitl --mavlink-url tcp:127.0.0.1:5760 --imu-topic /raw --gps-topic /gps/fix --odom-topic /r1a004/wheel_odom --cmd-vel-topic /cmd_vel_ardupilot --left-channel 1 --right-channel 3 --heading-topic /gps/heading
```
*Note:* After this connection is established, we can connect the Rover using Mavlink with GCS (QGroundControl / MissionPlanner) using a UDP connection.

### Terminal 5: Hybrid Navigation Nav2 Obstacle Avoidance Perform
**Description:** Builds the workspace and launches the local EKF odometry filter, `twist_mux`, Nav2 stack, `cmd_vel` EMA smoother, static TF publisher (`base_link` -> `laser`), and the Hybrid Executive node for automatic mode switching (AUTO <-> HOLD) and obstacle avoidance.
```bash
cd ardurover_ws
colcon build --symlink-install
source install/setup.bash 
ros2 launch hybrid_navigation hybrid_nav.launch.py
```

## Mission Control
* Connect the Rover with GCS (QGroundControl / MissionPlanner).
* Set a Mission in Mission Planner and Upload the Mission into the Rover.
* Arm the Rover Using Mission Planner.
* Change the mode into `AUTO` in Mission Planner.

## Working Principle
The Rover moves to the Mission Waypoints by using ArduRover, producing PWM Values. If an obstacle is present within 1m of the Rover, the ArduRover mode changes into `HOLD` mode and the Rover control is taken over by Nav2. It will set a local goal 2m in front of the rover, and the rover will move to the goal while avoiding the obstacle. After the local goal is reached, the ArduRover mode changes back into `AUTO` and continues the mission until it is complete.
