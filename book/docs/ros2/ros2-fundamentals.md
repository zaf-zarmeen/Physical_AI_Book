---
sidebar_position: 2
---

# ROS 2 Fundamentals: The Backbone of Physical AI

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is not an operating system but rather a flexible framework for writing robot software. It provides services designed for a heterogeneous computer cluster such as:

- Hardware abstraction
- Device drivers
- Libraries
- Visualizers
- Message-passing
- Package management

ROS 2 is critical for Physical AI because it enables modular robot development where different components (sensors, actuators, AI algorithms) can communicate seamlessly through standardized interfaces.

## ROS 2 Architecture

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. In Physical AI systems, nodes might represent:
- Sensor drivers (camera, LiDAR, IMU)
- Control algorithms (motor controllers)
- AI perception modules (object detection, SLAM)
- Planning systems (path planning, task scheduling)

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are data packets that follow a specific structure. For example:
- `/camera/image_raw`: carries image data from a camera
- `/cmd_vel`: carries velocity commands to a robot's base controller
- `/laser_scan`: carries distance measurements from a LiDAR

### Services
Services provide a request/reply communication pattern, useful for operations that have a clear beginning and end:
- `/set_parameters`: to configure node parameters
- `/spawn_entity`: to add objects to a simulation environment
- `/get_map`: to retrieve a pre-built map

### Actions
Actions are used for long-running tasks with feedback:
- `/navigate_to_pose`: to command robot navigation
- `/follow_waypoints`: to execute a sequence of navigation goals
- `/manipulation_task`: for complex robotic manipulation sequences

## Setting Up Your ROS 2 Environment

### Installing ROS 2 Humble Hawksbill

For Ubuntu 22.04:
```bash
# Setup locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
```

For other platforms, refer to the official ROS 2 documentation.

### Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your .bashrc to make permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Creating Your First ROS 2 Package

Let's create a simple package for Physical AI applications:

```bash
# Create a workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Create a package
colcon build
source install/setup.bash

# Create a new package
ros2 pkg create --build-type ament_python my_physical_ai_package
```

## Basic Publisher Example

Here's a simple publisher that could be used to send sensor data in a Physical AI system:

```python
# my_physical_ai_package/my_physical_ai_package/sensor_publisher.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import random

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'sensor_scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        
        # Simulate 360 distance measurements
        msg.ranges = [random.uniform(0.1, 10.0) for _ in range(360)]
        msg.angle_min = -3.14
        msg.angle_max = 3.14
        msg.angle_increment = 0.0174533  # 1 degree in radians
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {len(msg.ranges)} range measurements')


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = SensorPublisher()

    rclpy.spin(sensor_publisher)

    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Basic Subscriber Example

Here's a subscriber that processes the sensor data:

```python
# my_physical_ai_package/my_physical_ai_package/sensor_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'sensor_scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Process the sensor data
        # For example, find the minimum distance to an obstacle
        if msg.ranges:
            min_distance = min([r for r in msg.ranges if r > 0 and not r > 100])  # Filter invalid readings
            self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)

    sensor_subscriber = SensorSubscriber()

    rclpy.spin(sensor_subscriber)

    sensor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes with a single command:

```python
# my_physical_ai_package/launch/sensor_nodes.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_physical_ai_package',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen'
        ),
        Node(
            package='my_physical_ai_package',
            executable='sensor_subscriber',
            name='sensor_subscriber',
            output='screen'
        )
    ])
```

## Running the Example

1. Build your package:
```bash
cd ~/physical_ai_ws
colcon build --packages-select my_physical_ai_package
source install/setup.bash
```

2. Run the nodes separately:
```bash
# Terminal 1
ros2 run my_physical_ai_package sensor_publisher

# Terminal 2
ros2 run my_physical_ai_package sensor_subscriber
```

Or use the launch file:
```bash
ros2 launch my_physical_ai_package sensor_nodes.launch.py
```

## ROS 2 in Physical AI Context

In Physical AI systems, ROS 2 enables:

- **Modularity**: Different AI models and control systems can be developed separately and integrated
- **Reusability**: Standardized messages allow components to be reused across different robot platforms
- **Scalability**: Systems can run across multiple machines when needed
- **Simulation Integration**: Seamless transition between simulation and real hardware

## Diagram: ROS 2 Architecture
```
[Camera Node] ----> [Image Topic] ----> [Vision AI Node]
     |                     |
     v                     v
[Motor Controller] <-- [Command Topic] -- [Planning Node]
```

This diagram shows how different nodes communicate through topics, enabling the tight integration required for Physical AI systems.

## Exercises

### Exercise 2.1: Topic Exploration
Use the command line to explore existing ROS 2 topics on a system:
```bash
ros2 topic list
ros2 topic echo <topic_name>
```

### Exercise 2.2: Modify Publisher
Modify the publisher to simulate different types of sensors (e.g., ultrasound, infrared) with different characteristics.

### Exercise 2.3: Add Parameters
Add ROS 2 parameters to control the publishing rate and noise level of the sensor simulator.

## Mini Project 2.1: Physical AI Perception Node
Create a ROS 2 node that simulates a simple perception system. The node should:
1. Subscribe to a simulated camera topic (`/camera/image_raw`)
2. Process the image to detect a specific pattern (e.g., color blob, shape)
3. Publish the detection results to a new topic (`/detection_results`)
4. Include a parameter to configure what to detect

## Learning Objectives Review

By completing this chapter, you should now understand:

- The core concepts of ROS 2 architecture (nodes, topics, services, actions)
- How to create and build a ROS 2 package
- How to implement publishers and subscribers for sensor data
- How to use launch files to coordinate multiple nodes
- The role of ROS 2 in Physical AI systems