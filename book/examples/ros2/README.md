# ROS 2 Python Example: Simple Publisher/Subscriber

This directory contains basic ROS 2 Python examples for Physical AI applications.

## Getting Started

First, make sure you have ROS 2 Humble Hawksbill installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

## Basic Publisher Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Basic Subscriber Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Examples

1. Save the publisher code as `talker.py` and subscriber as `listener.py`

2. Make the files executable:
```bash
chmod +x talker.py
chmod +x listener.py
```

3. Open two terminals:
```bash
# Terminal 1 (publisher)
source /opt/ros/humble/setup.bash
python3 talker.py
```

```bash
# Terminal 2 (subscriber)
source /opt/ros/humble/setup.bash
python3 listener.py
```

## Physical AI Specific Example

Here's a more advanced example that might be used in Physical AI applications:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class PhysicalAIBotController(Node):

    def __init__(self):
        super().__init__('physical_ai_bot_controller')
        
        # Create subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create publisher for robot status
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Robot state variables
        self.laser_scan_data = None
        self.obstacle_detected = False
        self.min_distance = float('inf')
        
        self.get_logger().info("Physical AI Bot Controller initialized")

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_scan_data = msg
        self.process_scan_data()

    def process_scan_data(self):
        """Process the laser scan to detect obstacles"""
        if self.laser_scan_data is None:
            return
            
        # Get ranges from laser scan
        ranges = self.laser_scan_data.ranges
        
        # Filter out invalid ranges (inf or nan)
        valid_ranges = [r for r in ranges if not (math.isinf(r) or math.isnan(r))]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            self.obstacle_detected = self.min_distance < 1.0  # 1 meter threshold
        else:
            self.min_distance = float('inf')
            self.obstacle_detected = False

    def control_loop(self):
        """Main control loop"""
        cmd_vel_msg = Twist()
        
        if self.obstacle_detected:
            # Stop and turn if obstacle detected
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.5  # Turn right
            status_msg = String()
            status_msg.data = f"Obstacle detected at {self.min_distance:.2f}m, turning"
        else:
            # Move forward if no obstacles
            cmd_vel_msg.linear.x = 0.2  # 0.2 m/s forward
            cmd_vel_msg.angular.z = 0.0
            status_msg = String()
            status_msg.data = "Clear path, moving forward"
        
        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        # Publish status
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = PhysicalAIBotController()

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

This example demonstrates a basic robot controller that:
1. Subscribes to laser scan data
2. Processes the data to detect obstacles
3. Controls the robot's movement based on obstacle detection
4. Publishes status information

For use in Physical AI applications, this forms a foundation that can be extended with more sophisticated perception, planning, and control algorithms.