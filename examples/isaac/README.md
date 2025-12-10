# Isaac Sim Examples for Physical AI

This directory contains Isaac Sim examples as referenced in the textbook.

## Prerequisites

Before running these examples, ensure you have:

1. Isaac Sim installed via Omniverse
2. Python API access enabled
3. Required Python packages installed:
   ```bash
   pip install omniisaacgymenvs
   pip install torch
   ```

## Basic Isaac Sim Example

Here's a simple example showing how to set up a basic simulation environment in Isaac Sim:

```python
# basic_isaac_example.py

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
import numpy as np
import carb

# Initialize the simulation application
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros_bridge")

# Create a world instance
my_world = World(stage_units_in_meters=1.0)

# Get the assets root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets. Please check your installation.")
else:
    # Add a room environment
    add_reference_to_stage(
        usd_path="/Isaac/Environments/Simple_Room/simple_room.usd",
        prim_path="/World/simple_room"
    )

    # Add a robot to the stage
    my_robot = my_world.scene.add(
        Robot(
            prim_path="/World/Franka/franka",
            name="my_franka",
            usd_path=f"{assets_root_path}/Isaac/Robots/Franka/franka_instanceable.usd",
            position=np.array([0, 0, 0.052]),
            orientation=np.array([0, 0, 0, 1])
        )
    )

    # Add objects for manipulation
    my_world.scene.add(
        DynamicCuboid(
            prim_path="/World/cube_1",
            name="cube_1",
            position=np.array([0.5, 0.5, 0.1]),
            size=0.1,
            color=np.array([0.9, 0.1, 0.1])
        )
    )

    # Reset the world to apply all changes
    my_world.reset()

    # Main simulation loop
    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                my_world.reset()
                # Switch robot controller to position mode
                my_robot.get_articulation_controller().switch_control_mode("position")

            # Get robot position and orientation
            position, orientation = my_robot.get_world_pose()

            # Simple control logic could go here
            # For example, move the robot toward a target
            target_position = np.array([0.5, 0.5, 0.1])
            direction = target_position - position
            distance = np.linalg.norm(direction)

            if distance > 0.1:  # If farther than 10cm from target
                # Normalize direction and move slightly toward target
                direction = direction / distance * 0.01  # Small step
                # Apply this as a position command (simplified)
                # In practice, you'd use inverse kinematics to compute joint positions

simulation_app.close()
```

## Isaac Sim + ROS 2 Integration Example

For Physical AI applications, we often need to connect Isaac Sim to ROS 2:

```python
# isaac_ros_integration.py

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.sensor import Camera

# ROS 2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np

# Initialize ROS 2
rclpy.init()

class IsaacSimRosBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_ros_bridge')
        
        # Initialize world
        self.world = World(stage_units_in_meters=1.0)
        
        # Add reference to a simple environment
        add_reference_to_stage(
            usd_path="/Isaac/Environments/Simple_Room/simple_room.usd",
            prim_path="/World/simple_room"
        )
        
        # Add a simple robot
        self.my_robot = self.world.scene.add(
            Robot(
                prim_path="/World/Robot",
                name="my_robot",
                usd_path="/Isaac/Robots/Carter/carter.urdf",  # Example differential drive robot
                position=np.array([0.0, 0.0, 0.1]),
            )
        )
        
        # Add a camera to the robot
        self.camera = Camera(
            prim_path="/World/Robot/Camera",
            name="my_camera",
            position=np.array([0.2, 0, 0.1]),
            frequency=30,
            resolution=(640, 480)
        )
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # ROS 2 publishers and subscribers
        self.image_pub = self.create_publisher(Image, '/isaac/rgb_camera', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/isaac/scan', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Robot state
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Timer to publish sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        
        self.get_logger().info("Isaac Sim ROS Bridge initialized")

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from ROS 2"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        
        # Apply the command to the robot
        # In a real implementation, you'd interface with the Isaac Sim robot directly
        self.get_logger().info(f"Received cmd_vel: linear={self.linear_velocity}, angular={self.angular_velocity}")

    def publish_sensor_data(self):
        """Publish sensor data from Isaac Sim to ROS 2"""
        # Get image from Isaac Sim camera
        try:
            rgb_image = self.camera.get_rgb()
            if rgb_image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "rgb_camera"
                self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error publishing camera data: {e}")

    def run_simulation(self):
        """Run the main simulation loop"""
        while simulation_app.is_running():
            self.world.step(render=True)
            
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()

def main():
    # Create the ROS 2 node
    isaac_ros_bridge = IsaacSimRosBridge()
    
    # Run simulation in a separate thread
    import threading
    sim_thread = threading.Thread(target=isaac_ros_bridge.run_simulation)
    sim_thread.start()
    
    # Spin the ROS 2 node
    try:
        rclpy.spin(isaac_ros_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        simulation_app.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

## Usage Instructions

1. Make sure Isaac Sim is properly installed and running
2. Set up your Python environment with the required packages
3. Run the examples:

```bash
# For the basic example
python basic_isaac_example.py

# For the ROS2 integration example
python isaac_ros_integration.py
```

These examples demonstrate how Isaac Sim can be used for Physical AI applications, including perception, manipulation, and navigation tasks.