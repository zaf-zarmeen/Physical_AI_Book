---
sidebar_position: 1
---

# Hardware Setup Guide

## Required Hardware for Physical AI Development

This guide provides detailed instructions for setting up the hardware required to work with Physical AI and humanoid robotics systems.

## Robot Platforms

### Unitree Quadruped Robots
Unitree's A1, Go1, and H1 series robots are excellent platforms for Physical AI research:

- **Unitree A1**: Lightweight quadruped for research and development
  - Weight: 12 kg
  - Payload: 3 kg
  - Max speed: 3 m/s
  - Battery life: 2+ hours
  - Onboard computing: NVIDIA Jetson NX

- **Unitree Go1**: More robust quadruped for advanced applications
  - Weight: 13 kg
  - Payload: 5 kg
  - Max speed: 3 m/s
  - Battery life: 2+ hours
  - Improved joint servos

- **Unitree H1**: Humanoid robot platform
  - Weight: 47 kg
  - Payload: 5 kg
  - Height: 1.05 m
  - Battery life: 2+ hours
  - 25 DOF (degrees of freedom)

### Setting Up Unitree Robots
1. **Initial Setup**
   - Connect to the robot via Ethernet or Wi-Fi
   - Update firmware to the latest version
   - Calibrate IMU and joint encoders

2. **ROS 2 Interface**
   - Install Unitree ROS 2 packages
   - Configure network settings for real-time communication
   - Test basic movement commands

### Other Compatible Platforms
- **TurtleBot3**: Great for beginners
- **Stretch RE1**: Mobile manipulator platform
- **ANYmal**: Industrial-grade quadruped
- **Boston Dynamics Spot**: Research platform (requires special licensing)

## Sensor Hardware

### Intel RealSense Cameras
Intel RealSense cameras provide depth sensing for Physical AI systems:

- **D435/D435i**: Stereo depth cameras
  - Depth resolution: up to 1280×720
  - RGB resolution: up to 1920×1080
  - FOV: 90° (H) × 58° (V) × 105° (D)
  - Connect via USB 3.0

- **L515**: LiDAR-based depth camera
  - Range: 0.25m to 9m
  - Resolution: up to 1024×768
  - Less affected by lighting conditions

### IMU Sensors
Inertial Measurement Units are critical for robot stability:

- **Recommended**: Bosch BNO055 or Xsens MTi series
- Required for: Balance, orientation, motion tracking
- Integration with robot state publisher

### LiDAR Sensors
- **Hokuyo UST-10LX**: 2D LiDAR with 270° field of view
- **Velodyne VLP-16**: 3D LiDAR for full 3D environment mapping
- **SICK TIM5XX**: Industrial-grade 2D LiDAR

## Computing Hardware

### NVIDIA Jetson Series
For edge AI computing on robots:

- **Jetson Nano**: Basic AI computing
  - 128-core NVIDIA Maxwell GPU
  - 4GB LPDDR4
  - 40-pin GPIO header
  - Suitable for basic perception tasks

- **Jetson Xavier NX**: Mid-tier AI performance
  - 384-core NVIDIA Volta GPU with Tensor Cores
  - 8GB LPDDR4x
  - Up to 21 TOPS AI performance
  - Suitable for real-time perception and planning

- **Jetson AGX Orin**: High-performance AI
  - 2048-core NVIDIA Ampere GPU
  - Up to 64GB LPDDR5
  - Up to 275 TOPS AI performance
  - Suitable for complex VLA systems

### Laptop Requirements for Development
For simulation and development work:

- **CPU**: Intel i7 or AMD Ryzen 7 (8+ cores)
- **RAM**: 32GB or more
- **GPU**: NVIDIA RTX 3070 or better
- **Storage**: 1TB SSD (500GB minimum)
- **OS**: Ubuntu 22.04 LTS or Windows 11 with WSL2

## Network Setup

### Real-time Communication
For robot control, ensure low-latency networking:

- **Wi-Fi 6**: For wireless robot communication
  - Minimum 802.11ax standard
  - 5GHz band preferred
  - Quality of Service (QoS) settings

- **Ethernet**: For critical control loops
  - Gigabit or better
  - Direct connection when possible
  - Deterministic communication

### Network Configuration
1. Set up a dedicated robot network (192.168.1.x)
2. Configure static IPs for robots
3. Set up VPN for remote access if needed

## Safety Equipment

### Physical Safety
- Emergency stop buttons accessible to operators
- Safety barriers around robot workspaces
- Proper lighting in robot areas
- Clear pathways for human traffic

### Electronic Safety
- Fuses and circuit breakers on power systems
- Proper grounding of all equipment
- UPS (Uninterruptible Power Supply) for critical systems

## Initial Setup Checklist

- [ ] Robot mechanically assembled and calibrated
- [ ] All sensors connected and configured
- [ ] Computing platform installed and updated
- [ ] ROS 2 environment configured
- [ ] Network communication tested
- [ ] Basic movement commands verified
- [ ] Safety measures in place
- [ ] Backup and recovery procedures established

## Troubleshooting Common Issues

### Robot Communication Problems
- Check network connections and IP configurations
- Verify firewall settings on both robot and development machine
- Test communication with ping and network diagnostic tools

### Sensor Calibration Issues
- Follow manufacturer's calibration procedures
- Ensure proper lighting conditions for cameras
- Check for sensor interference

### Performance Issues
- Monitor CPU and GPU usage
- Optimize ROS 2 node configurations
- Consider distributed computing if needed

## Next Steps
After completing hardware setup, proceed to:
1. Software installation and configuration
2. Basic robot operation tutorials
3. Integration with simulation environments