# Gazebo Examples for Physical AI

This directory contains Gazebo simulation examples as referenced in the textbook.

## Prerequisites

Before running these examples, ensure you have:

1. Gazebo Garden or newer installed
2. ROS 2 Humble Hawksbill
3. Gazebo ROS 2 packages:
   ```bash
   sudo apt install ros-humble-gazebo-ros2-control ros-humble-gazebo-plugins
   ```

## Basic Gazebo World File

Create a simple world file for Physical AI testing:

```xml
<!-- physical_ai_test.world -->
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

## Launch File for Gazebo

Create a launch file to start Gazebo with the ROS 2 bridge:

```python
# gazebo_simulation.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', 
             PathJoinSubstitution([FindPackageShare('my_physical_ai_package'), 
                                  'worlds', 'physical_ai_test.world'])],
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

## Gazebo ROS 2 Controller Example

Here's an example of a ROS 2 node that controls a robot in Gazebo:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class GazeboRobotController(Node):

    def __init__(self):
        super().__init__('gazebo_robot_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Subscriber for odometry data
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Robot state variables
        self.laser_data = None
        self.odom_data = None
        self.obstacle_detected = False
        self.min_distance = float('inf')
        
        self.get_logger().info("Gazebo Robot Controller initialized")

    def scan_callback(self, msg):
        """Process laser scan data from Gazebo"""
        self.laser_data = msg
        self.process_laser_data()

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg

    def process_laser_data(self):
        """Process laser scan to detect obstacles"""
        if self.laser_data is None:
            return
            
        # Process the scan data to find minimum distance
        ranges = self.laser_data.ranges
        
        # Filter out invalid ranges
        valid_ranges = [r for r in ranges if r > self.laser_data.range_min and r < self.laser_data.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            self.obstacle_detected = self.min_distance < 0.8  # 0.8m threshold
        else:
            self.min_distance = float('inf')
            self.obstacle_detected = False

    def control_loop(self):
        """Main control loop for robot navigation in Gazebo"""
        cmd_vel_msg = Twist()
        
        if self.obstacle_detected:
            # Obstacle detected - stop and turn
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.5  # Turn right
            self.get_logger().info(f"Obstacle detected at {self.min_distance:.2f}m, turning")
        else:
            # No obstacles - move forward
            cmd_vel_msg.linear.x = 0.3  # 0.3 m/s forward
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info("Clear path, moving forward")
        
        # Publish the velocity command
        self.cmd_vel_publisher.publish(cmd_vel_msg)


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

## URDF Model Example

Create a simple robot URDF model for use in Gazebo:

```xml
<!-- simple_robot.urdf -->
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Front caster wheel -->
  <joint name="caster_front_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_front"/>
    <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
  </joint>

  <link name="caster_front">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.3 -0.1" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.3 -0.1" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Camera sensor -->
  <gazebo reference="base_link">
    <sensor name="camera" type="camera">
      <pose>0.2 0 0.1 0 0 0</pose>
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
  </gazebo>

  <!-- Laser sensor -->
  <gazebo reference="base_link">
    <sensor name="laser" type="ray">
      <pose>0.2 0 0.2 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- Differential drive controller -->
  <gazebo>
    <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive">
      <commandTopic>/cmd_vel</commandTopic>
      <odometryTopic>/odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
      <publishOdomTF>true</publishOdomTF>
      <wheelSeparation>0.6</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <maxWheelTorque>20</maxWheelTorque>
      <maxWheelAcceleration>1.0</maxWheelAcceleration>
    </plugin>
  </gazebo>

</robot>
```

## Running the Examples

1. Place the world file in `~/.gazebo/worlds/`:
   ```bash
   cp physical_ai_test.world ~/.gazebo/worlds/
   ```

2. Launch Gazebo with your world:
   ```bash
   gazebo --verbose ~/.gazebo/worlds/physical_ai_test.world
   ```

3. Or use the ROS 2 launch file:
   ```bash
   ros2 launch gazebo_simulation.launch.py
   ```

4. Run the controller:
   ```bash
   ros2 run my_package gazebo_robot_controller.py
   ```

## Creating a Gazebo Package

To create a proper ROS 2 package for Gazebo simulation:

1. Create the package:
   ```bash
   ros2 pkg create --build-type ament_python my_gazebo_simulation
   ```

2. Add the launch files, URDF models, and world files to the appropriate directories

3. Update the package.xml to include Gazebo dependencies

4. Build and run the package with:
   ```bash
   colcon build --packages-select my_gazebo_simulation
   source install/setup.bash
   ros2 launch my_gazebo_simulation gazebo_simulation.launch.py
   ```

These examples provide a foundation for developing Physical AI applications in Gazebo.