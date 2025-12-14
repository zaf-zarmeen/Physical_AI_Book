---
sidebar_position: 3
---

# Gazebo Physics Simulation: Testing Physical AI Systems

## Introduction to Gazebo

Gazebo is a physics-based simulation environment that provides realistic sensor simulation and dynamics for Physical AI development. It allows you to test and validate your Physical AI algorithms in a safe, controlled environment before deploying to real robots.

Gazebo is particularly important in Physical AI because:
- It provides realistic physics simulation for robot behaviors
- It simulates various sensors (cameras, LiDAR, IMUs) with realistic noise models
- It allows rapid testing of complex scenarios without hardware risk
- It supports a wide range of robot models and environments

## Installing Gazebo Garden

For Ubuntu 22.04:
```bash
# Setup sources
sudo apt update && sudo apt install wget
sudo sh -c 'echo "deb [arch=amd64] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Gazebo Garden
sudo apt update
sudo apt install gazebo
```

## Basic Gazebo Concepts

### Worlds
Worlds define the 3D environment where simulation occurs. They contain:
- Models (robots, objects, obstacles)
- Physics properties (gravity, friction)
- Light sources
- Terrain and environment

### Models
Models represent physical objects in the simulation. They include:
- Visual geometry (what the model looks like)
- Collision geometry (how it interacts physically)
- Inertia properties
- Sensors
- Joints and actuators

### Plugins
Plugins extend Gazebo's functionality:
- Sensor plugins to simulate various sensors
- Controller plugins to interface with ROS 2
- Physics plugins to customize physical interactions

## Connecting Gazebo to ROS 2

Gazebo connects to ROS 2 through the Gazebo ROS 2 Bridge package, which creates ROS 2 interfaces for Gazebo's native communication.

### Installing Gazebo ROS 2 Bridge
```bash
sudo apt install ros-humble-gazebo-ros2-control ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

## Creating a Simple World

Let's create a simple world file for Physical AI testing:

```xml
<!-- ~/.gazebo/worlds/physical_ai_test.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physical_ai_test">

    <!-- Include a model from Gazebo's model database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A simple box obstacle -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Our test robot -->
    <model name="test_robot">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="chassis">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.3</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>0.4167</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.4167</iyy>
            <iyz>0</iyz>
            <izz>0.4167</izz>
          </inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

## Launching Gazebo with ROS 2 Bridge

Create a launch file to start Gazebo with the ROS 2 bridge:

```python
# my_physical_ai_package/launch/gazebo_simulation.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start Gazebo with a world file
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so', 
             PathJoinSubstitution([FindPackageShare('my_physical_ai_package'), 'worlds', 'physical_ai_test.world'])],
        output='screen'
    )
    
    # Start Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client
    ])
```

## Adding a Camera Sensor

Let's extend our robot with a camera sensor for Physical AI perception:

```xml
<!-- Add this inside the test_robot model -->
<model name="test_robot">
  <pose>0 0 0.5 0 0 0</pose>
  
  <!-- The chassis link -->
  <link name="chassis">
    <!-- ... (previous chassis content) ... -->
    
    <!-- Camera sensor -->
    <sensor name="camera" type="camera">
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
  </link>
</model>
```

## Working with Gazebo and ROS 2

When Gazebo is connected to ROS 2, you can control simulation and access sensor data through ROS 2 topics:

### Controlling the Robot

```python
# my_physical_ai_package/my_physical_ai_package/gazebo_robot_controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class GazeboRobotController(Node):

    def __init__(self):
        super().__init__('gazebo_robot_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for camera images
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # State control variables
        self.obstacle_detected = False
        
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Simple obstacle detection (for example purposes)
        # In a real Physical AI system, this would use more sophisticated perception
        height, width, _ = cv_image.shape
        center_region = cv_image[height//2-50:height//2+50, width//2-50:width//2+50]
        
        # Detect red obstacles (in our example world)
        hsv = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        
        mask = mask1 + mask2
        red_pixels = cv2.countNonZero(mask)
        
        # Set obstacle detection flag
        self.obstacle_detected = red_pixels > 100  # Threshold for detection
        
        # Display the image with detection
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def control_loop(self):
        msg = Twist()
        
        if self.obstacle_detected:
            # Stop or turn when obstacle detected
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Turn right
        else:
            # Move forward when no obstacles
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = GazeboRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Simulation

1. Launch the Gazebo simulation:
```bash
# In the workspace root
source install/setup.bash
ros2 launch my_physical_ai_package gazebo_simulation.launch.py
```

2. In a separate terminal, run the robot controller:
```bash
ros2 run my_physical_ai_package gazebo_robot_controller
```

## Advanced Integration: Unity Visualization

Gazebo excels at physics simulation, but for intuitive visualization and human-robot interaction, Physical AI systems often integrate with Unity. Unity provides:

- High-quality visualization
- Intuitive interface for human operators
- VR/AR capabilities for immersive control
- Realistic rendering for synthetic data generation

The connection between Gazebo physics and Unity visualization is typically achieved through:
- ROS 2 bridges to synchronize state between simulators
- Custom plugins to convert data formats
- Network connections for distributed simulation

## Best Practices for Physical AI Simulations

1. **Realistic Sensor Models**: Use sensor plugins that accurately simulate real-world noise and limitations
2. **Physics Validation**: Validate simulation physics against real-world measurements
3. **Domain Randomization**: Vary simulation parameters to create robust Physical AI systems
4. **Sim-to-Real Transfer**: Design systems that can adapt from simulation to real robots
5. **Safety Boundaries**: Implement safety checks even in simulation to prevent harmful behaviors

## Diagram: Gazebo Integration with Physical AI System
```
[Real Robot]  <--->  [ROS 2]  <--->  [Gazebo Simulator]  <--> [Unity Visualizer]
     ^                   |                   |                     |
     |-------------- [Real-World Data] ---- [Physics Simulation] --|
```

This diagram illustrates how Gazebo serves as the physics simulation layer in a Physical AI system, connected to both real robots and other visualization tools through ROS 2.

## Exercises

### Exercise 3.1: World Creation
Create a new world file with multiple obstacles and a simple maze structure for navigation testing.

### Exercise 3.2: Sensor Integration
Add a LiDAR sensor to your robot model and implement LiDAR-based obstacle detection in your controller.

### Exercise 3.3: Physics Tuning
Experiment with different physics properties (friction, restitution) to see how they affect robot behavior.

## Mini Project 3.1: Gazebo Physical AI Challenge
Create a complete simulation environment with:
1. A differential drive robot with camera and LiDAR
2. A structured environment with obstacles
3. A ROS 2 node that uses both camera and LiDAR data for navigation
4. A safety system that prevents collisions
5. Performance metrics tracking (time to goal, collisions, etc.)

## Learning Objectives Review

By completing this chapter, you should now understand:

- How to create and configure Gazebo simulation environments
- How to connect Gazebo to ROS 2 for Physical AI development
- How to add sensors to simulation models
- How to access sensor data and control robots through ROS 2
- The role of simulation in developing safe, reliable Physical AI systems